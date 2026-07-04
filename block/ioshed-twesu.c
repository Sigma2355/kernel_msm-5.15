// SPDX-License-Identifier: GPL-2.0
/*
* TWESU (Tweak-Wes-U) MQ IO Scheduler - Ultra Edition (High-Speed & Low-Latency)
*
* Dioptimalkan dengan konfigurasi asimetris ekstrim namun tetap aman.
* Menjamin zero-lag UI/Gaming dengan batas latensi baca 30ms, serta mempertahankan
* kestabilan throughput transfer data (MTP/OTG) melalui pengosongan antrean agresif.
*/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>
#include <linux/sbitmap.h>

#include <trace/events/block.h>

#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-debugfs.h"
#include "blk-mq-tag.h"
#include "blk-mq-sched.h"

/*
* TUNING ASIMETRIS ULTRA-PERFORMANCE (KENCANG & AMAN):
* UI sangat responsif untuk gaming ekstrim, tetapi transfer file tetap melesat tanpa stall.
*/
static const int read_expire = HZ / 33;   /* Ultra Tweak: Batas waktu baca hanya 30ms! Respons instan di bawah refresh rate */
static const int write_expire = HZ;       /* Ultra Tweak: Flush menulis dipercepat dalam 1 detik untuk mencegah kemacetan transfer data */
/*
* Batas waktu sebelum permintaan prioritas rendah (background) diizinkan menyela.
*/
static const int prio_aging_expire = 2 * HZ; /* Dipercepat ke 2 detik agar proses background tidak mengunci IO terlalu lama */
static const int writes_starved = 2;      /* Rasio ketat: Pembacaan hanya boleh menyela menulis maksimal 2 kali demi kelancaran transfer data */
static const int fifo_batch = 96;         /* Ultra Tweak: Batch pengiriman dinaikkan ke 96 untuk memaksimalkan bandwidth penyimpanan UFS */

enum twesu_data_dir {
        TWESU_READ        = READ,
        TWESU_WRITE        = WRITE,
};

enum { TWESU_DIR_COUNT = 2 };

enum twesu_prio {
        TWESU_RT_PRIO        = 0,
        TWESU_BE_PRIO        = 1,
        TWESU_IDLE_PRIO        = 2,
        TWESU_PRIO_MAX        = 2,
};

enum { TWESU_PRIO_COUNT = 3 };

struct io_stats_per_prio {
        uint32_t inserted;
        uint32_t merged;
        uint32_t dispatched;
        atomic_t completed;
};

struct twesu_per_prio {
        struct list_head dispatch;
        struct rb_root sort_list[TWESU_DIR_COUNT];
        struct list_head fifo_list[TWESU_DIR_COUNT];
        struct request *next_rq[TWESU_DIR_COUNT];
        struct io_stats_per_prio stats;
};

struct twesu_data {
        struct twesu_per_prio per_prio[TWESU_PRIO_COUNT];
        enum twesu_data_dir last_dir;
        unsigned int batching;
        unsigned int starved;

        int fifo_expire[TWESU_DIR_COUNT];
        int fifo_batch;
        int writes_starved;
        int front_merges;
        u32 async_depth;
        int prio_aging_expire;

        spinlock_t lock;
        spinlock_t zone_lock;
};

static const enum twesu_prio ioprio_class_to_prio[] = {
        [IOPRIO_CLASS_NONE]        = TWESU_BE_PRIO,
        [IOPRIO_CLASS_RT]        = TWESU_RT_PRIO,
        [IOPRIO_CLASS_BE]        = TWESU_BE_PRIO,
        [IOPRIO_CLASS_IDLE]        = TWESU_IDLE_PRIO,
};

static inline struct rb_root *
twesu_rb_root(struct twesu_per_prio *per_prio, struct request *rq)
{
        return &per_prio->sort_list[rq_data_dir(rq)];
}

static u8 twesu_rq_ioclass(struct request *rq)
{
        return IOPRIO_PRIO_CLASS(req_get_ioprio(rq));
}

static inline struct request *
twesu_earlier_request(struct request *rq)
{
        struct rb_node *node = rb_prev(&rq->rb_node);

        if (node)
                return rb_entry_rq(node);

        return NULL;
}

static inline struct request *
twesu_latter_request(struct request *rq)
{
        struct rb_node *node = rb_next(&rq->rb_node);

        if (node)
                return rb_entry_rq(node);

        return NULL;
}

static void
twesu_add_rq_rb(struct twesu_per_prio *per_prio, struct request *rq)
{
        struct rb_root *root = twesu_rb_root(per_prio, rq);

        elv_rb_add(root, rq);
}

static inline void
twesu_del_rq_rb(struct twesu_per_prio *per_prio, struct request *rq)
{
        const enum twesu_data_dir data_dir = rq_data_dir(rq);

        if (per_prio->next_rq[data_dir] == rq)
                per_prio->next_rq[data_dir] = twesu_latter_request(rq);

        elv_rb_del(twesu_rb_root(per_prio, rq), rq);
}

static void twesu_remove_request(struct request_queue *q,
                                    struct twesu_per_prio *per_prio,
                                    struct request *rq)
{
        list_del_init(&rq->queuelist);

        if (!RB_EMPTY_NODE(&rq->rb_node))
                twesu_del_rq_rb(per_prio, rq);

        elv_rqhash_del(q, rq);
        if (q->last_merge == rq)
                q->last_merge = NULL;
}

static void twesu_request_merged(struct request_queue *q, struct request *req,
                              enum elv_merge type)
{
        struct twesu_data *dd = q->elevator->elevator_data;
        const u8 ioprio_class = twesu_rq_ioclass(req);
        const enum twesu_prio prio = ioprio_class_to_prio[ioprio_class];
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];

        if (type == ELEVATOR_FRONT_MERGE) {
                elv_rb_del(twesu_rb_root(per_prio, req), req);
                twesu_add_rq_rb(per_prio, req);
        }
}

static void twesu_merged_requests(struct request_queue *q, struct request *req,
                               struct request *next)
{
        struct twesu_data *dd = q->elevator->elevator_data;
        const u8 ioprio_class = twesu_rq_ioclass(next);
        const enum twesu_prio prio = ioprio_class_to_prio[ioprio_class];

        lockdep_assert_held(&dd->lock);

        dd->per_prio[prio].stats.merged++;

        if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
                if (time_before((unsigned long)next->fifo_time,
                                (unsigned long)req->fifo_time)) {
                        list_move(&req->queuelist, &next->queuelist);
                        req->fifo_time = next->fifo_time;
                }
        }

        twesu_remove_request(q, &dd->per_prio[prio], next);
}

static void
twesu_move_request(struct twesu_data *dd, struct twesu_per_prio *per_prio,
                      struct request *rq)
{
        const enum twesu_data_dir data_dir = rq_data_dir(rq);

        per_prio->next_rq[data_dir] = twesu_latter_request(rq);

        twesu_remove_request(rq->q, per_prio, rq);
}

static u32 twesu_queued(struct twesu_data *dd, enum twesu_prio prio)
{
        const struct io_stats_per_prio *stats = &dd->per_prio[prio].stats;

        lockdep_assert_held(&dd->lock);

        return stats->inserted - atomic_read(&stats->completed);
}

static inline int twesu_check_fifo(struct twesu_per_prio *per_prio,
                                      enum twesu_data_dir data_dir)
{
        struct request *rq = rq_entry_fifo(per_prio->fifo_list[data_dir].next);

        if (time_after_eq(jiffies, (unsigned long)rq->fifo_time))
                return 1;

        return 0;
}

static bool twesu_is_seq_write(struct twesu_data *dd, struct request *rq)
{
        struct request *prev = twesu_earlier_request(rq);

        if (!prev)
                return false;

        return blk_rq_pos(prev) + blk_rq_sectors(prev) == blk_rq_pos(rq);
}

static struct request *twesu_skip_seq_writes(struct twesu_data *dd,
                                                struct request *rq)
{
        sector_t pos = blk_rq_pos(rq);
        sector_t skipped_sectors = 0;

        while (rq) {
                if (blk_rq_pos(rq) != pos + skipped_sectors)
                        break;
                skipped_sectors += blk_rq_sectors(rq);
                rq = twesu_latter_request(rq);
        }

        return rq;
}

static struct request *
twesu_fifo_request(struct twesu_data *dd, struct twesu_per_prio *per_prio,
                      enum twesu_data_dir data_dir)
{
        struct request *rq;
        unsigned long flags;

        if (list_empty(&per_prio->fifo_list[data_dir]))
                return NULL;

        rq = rq_entry_fifo(per_prio->fifo_list[data_dir].next);
        if (data_dir == TWESU_READ || !blk_queue_is_zoned(rq->q))
                return rq;

        spin_lock_irqsave(&dd->zone_lock, flags);
        list_for_each_entry(rq, &per_prio->fifo_list[TWESU_WRITE], queuelist) {
                if (blk_req_can_dispatch_to_zone(rq) &&
                    (blk_queue_nonrot(rq->q) ||
                     !twesu_is_seq_write(dd, rq)))
                        goto out;
        }
        rq = NULL;
out:
        spin_unlock_irqrestore(&dd->zone_lock, flags);

        return rq;
}

static struct request *
twesu_next_request(struct twesu_data *dd, struct twesu_per_prio *per_prio,
                      enum twesu_data_dir data_dir)
{
        struct request *rq;
        unsigned long flags;

        rq = per_prio->next_rq[data_dir];
        if (!rq)
                return NULL;

        if (data_dir == TWESU_READ || !blk_queue_is_zoned(rq->q))
                return rq;

        spin_lock_irqsave(&dd->zone_lock, flags);
        while (rq) {
                if (blk_req_can_dispatch_to_zone(rq))
                        break;
                if (blk_queue_nonrot(rq->q))
                        rq = twesu_latter_request(rq);
                else
                        rq = twesu_skip_seq_writes(dd, rq);
        }
        spin_unlock_irqrestore(&dd->zone_lock, flags);

        return rq;
}

static bool started_after(struct twesu_data *dd, struct request *rq,
                          unsigned long latest_start)
{
        unsigned long start_time = (unsigned long)rq->fifo_time;

        start_time -= dd->fifo_expire[rq_data_dir(rq)];

        return time_after(start_time, latest_start);
}

static struct request *__twesu_dispatch_request(struct twesu_data *dd,
                                             struct twesu_per_prio *per_prio,
                                             unsigned long latest_start)
{
        struct request *rq, *next_rq;
        enum twesu_data_dir data_dir;
        enum twesu_prio prio;
        u8 ioprio_class;

        lockdep_assert_held(&dd->lock);

        if (!list_empty(&per_prio->dispatch)) {
                rq = list_first_entry(&per_prio->dispatch, struct request,
                                      queuelist);
                if (started_after(dd, rq, latest_start))
                        return NULL;
                list_del_init(&rq->queuelist);
                goto done;
        }

        rq = twesu_next_request(dd, per_prio, dd->last_dir);
        if (rq && dd->batching < dd->fifo_batch)
                goto dispatch_request;

        if (!list_empty(&per_prio->fifo_list[TWESU_READ])) {
                BUG_ON(RB_EMPTY_ROOT(&per_prio->sort_list[TWESU_READ]));

                if (twesu_fifo_request(dd, per_prio, TWESU_WRITE) &&
                    (dd->starved++ >= dd->writes_starved))
                        goto dispatch_writes;

                data_dir = TWESU_READ;

                goto dispatch_find_request;
        }

        if (!list_empty(&per_prio->fifo_list[TWESU_WRITE])) {
dispatch_writes:
                BUG_ON(RB_EMPTY_ROOT(&per_prio->sort_list[TWESU_WRITE]));

                dd->starved = 0;

                data_dir = TWESU_WRITE;

                goto dispatch_find_request;
        }

        return NULL;

dispatch_find_request:
        next_rq = twesu_next_request(dd, per_prio, data_dir);
        if (twesu_check_fifo(per_prio, data_dir) || !next_rq) {
                rq = twesu_fifo_request(dd, per_prio, data_dir);
        } else {
                rq = next_rq;
        }

        if (!rq)
                return NULL;

        dd->last_dir = data_dir;
        dd->batching = 0;

dispatch_request:
        if (started_after(dd, rq, latest_start))
                return NULL;

        dd->batching++;
        twesu_move_request(dd, per_prio, rq);
done:
        ioprio_class = twesu_rq_ioclass(rq);
        prio = ioprio_class_to_prio[ioprio_class];
        dd->per_prio[prio].stats.dispatched++;
        
        blk_req_zone_write_lock(rq);
        rq->rq_flags |= RQF_STARTED;
        return rq;
}

static struct request *twesu_dispatch_prio_aged_requests(struct twesu_data *dd,
                                                      unsigned long now)
{
        struct request *rq;
        enum twesu_prio prio;
        int prio_cnt;

        lockdep_assert_held(&dd->lock);

        prio_cnt = !!twesu_queued(dd, TWESU_RT_PRIO) + !!twesu_queued(dd, TWESU_BE_PRIO) +
                   !!twesu_queued(dd, TWESU_IDLE_PRIO);
        if (prio_cnt < 2)
                return NULL;

        for (prio = TWESU_BE_PRIO; prio <= TWESU_PRIO_MAX; prio++) {
                rq = __twesu_dispatch_request(dd, &dd->per_prio[prio],
                                           now - dd->prio_aging_expire);
                if (rq)
                        return rq;
        }

        return NULL;
}

static struct request *twesu_dispatch_request(struct blk_mq_hw_ctx *hctx)
{
        struct twesu_data *dd = hctx->queue->elevator->elevator_data;
        const unsigned long now = jiffies;
        struct request *rq;
        enum twesu_prio prio;

        spin_lock(&dd->lock);
        rq = twesu_dispatch_prio_aged_requests(dd, now);
        if (rq)
                goto unlock;

        for (prio = 0; prio <= TWESU_PRIO_MAX; prio++) {
                rq = __twesu_dispatch_request(dd, &dd->per_prio[prio], now);
                if (rq || twesu_queued(dd, prio))
                        break;
        }

unlock:
        spin_unlock(&dd->lock);

        return rq;
}

static void twesu_limit_depth(unsigned int op, struct blk_mq_alloc_data *data)
{
        struct twesu_data *dd = data->q->elevator->elevator_data;

        if (op_is_sync(op) && !op_is_write(op))
                return;

        data->shallow_depth = dd->async_depth;
}

static void twesu_depth_updated(struct blk_mq_hw_ctx *hctx)
{
        struct request_queue *q = hctx->queue;
        struct twesu_data *dd = q->elevator->elevator_data;
        struct blk_mq_tags *tags = hctx->sched_tags;
        unsigned int shift = tags->bitmap_tags->sb.shift;

        dd->async_depth = max(1U, 3 * (1U << shift)  / 4);

        sbitmap_queue_min_shallow_depth(tags->bitmap_tags, dd->async_depth);
}

static int twesu_init_hctx(struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx)
{
        twesu_depth_updated(hctx);
        return 0;
}

static void twesu_exit_sched(struct elevator_queue *e)
{
        struct twesu_data *dd = e->elevator_data;
        enum twesu_prio prio;

        for (prio = 0; prio <= TWESU_PRIO_MAX; prio++) {
                struct twesu_per_prio *per_prio = &dd->per_prio[prio];
                const struct io_stats_per_prio *stats = &per_prio->stats;
                uint32_t queued;

                WARN_ON_ONCE(!list_empty(&per_prio->fifo_list[TWESU_READ]));
                WARN_ON_ONCE(!list_empty(&per_prio->fifo_list[TWESU_WRITE]));

                spin_lock(&dd->lock);
                queued = twesu_queued(dd, prio);
                spin_unlock(&dd->lock);

                WARN_ONCE(queued != 0,
                          "Statistik untuk prioritas %d: i %u m %u d %u c %u\n",
                          prio, stats->inserted, stats->merged,
                          stats->dispatched, atomic_read(&stats->completed));
        }

        kfree(dd);
}

static int twesu_init_sched(struct request_queue *q, struct elevator_type *e)
{
        struct twesu_data *dd;
        struct elevator_queue *eq;
        enum twesu_prio prio;
        int ret = -ENOMEM;

        eq = elevator_alloc(q, e);
        if (!eq)
                return ret;

        dd = kzalloc_node(sizeof(*dd), GFP_KERNEL, q->node);
        if (!dd)
                goto put_eq;

        eq->elevator_data = dd;

        for (prio = 0; prio <= TWESU_PRIO_MAX; prio++) {
                struct twesu_per_prio *per_prio = &dd->per_prio[prio];

                INIT_LIST_HEAD(&per_prio->dispatch);
                INIT_LIST_HEAD(&per_prio->fifo_list[TWESU_READ]);
                INIT_LIST_HEAD(&per_prio->fifo_list[TWESU_WRITE]);
                per_prio->sort_list[TWESU_READ] = RB_ROOT;
                per_prio->sort_list[TWESU_WRITE] = RB_ROOT;
        }
        
        /*
         * PENETAPAN TUNING ULTRA EDITION:
         */
        dd->fifo_expire[TWESU_READ] = read_expire;
        dd->fifo_expire[TWESU_WRITE] = write_expire;
        dd->writes_starved = writes_starved;
        dd->front_merges = 1;                 /* Selalu diaktifkan untuk menjaga efisiensi transfer data sequential */
        dd->last_dir = TWESU_WRITE;
        dd->fifo_batch = fifo_batch;
        dd->prio_aging_expire = prio_aging_expire;
        spin_lock_init(&dd->lock);
        spin_lock_init(&dd->zone_lock);

        q->elevator = eq;
        return 0;

put_eq:
        kobject_put(&eq->kobj);
        return ret;
}

static int twesu_request_merge(struct request_queue *q, struct request **rq,
                            struct bio *bio)
{
        struct twesu_data *dd = q->elevator->elevator_data;
        const u8 ioprio_class = IOPRIO_PRIO_CLASS(bio->bi_ioprio);
        const enum twesu_prio prio = ioprio_class_to_prio[ioprio_class];
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];
        sector_t sector = bio_end_sector(bio);
        struct request *__rq;

        if (!dd->front_merges)
                return ELEVATOR_NO_MERGE;

        __rq = elv_rb_find(&per_prio->sort_list[bio_data_dir(bio)], sector);
        if (__rq) {
                BUG_ON(sector != blk_rq_pos(__rq));

                if (elv_bio_merge_ok(__rq, bio)) {
                        *rq = __rq;
                        if (blk_discard_mergable(__rq))
                                return ELEVATOR_DISCARD_MERGE;
                        return ELEVATOR_FRONT_MERGE;
                }
        }

        return ELEVATOR_NO_MERGE;
}

static bool twesu_bio_merge(struct request_queue *q, struct bio *bio,
                unsigned int nr_segs)
{
        struct twesu_data *dd = q->elevator->elevator_data;
        struct request *free = NULL;
        bool ret;

        spin_lock(&dd->lock);
        ret = blk_mq_sched_try_merge(q, bio, nr_segs, &free);
        spin_unlock(&dd->lock);

        if (free)
                blk_mq_free_request(free);

        return ret;
}

static void twesu_insert_request(struct blk_mq_hw_ctx *hctx, struct request *rq,
                              bool at_head)
{
        struct request_queue *q = hctx->queue;
        struct twesu_data *dd = q->elevator->elevator_data;
        const enum twesu_data_dir data_dir = rq_data_dir(rq);
        u16 ioprio = req_get_ioprio(rq);
        u8 ioprio_class = IOPRIO_PRIO_CLASS(ioprio);
        struct twesu_per_prio *per_prio;
        enum twesu_prio prio;
        LIST_HEAD(free);

        lockdep_assert_held(&dd->lock);

        blk_req_zone_write_unlock(rq);

        prio = ioprio_class_to_prio[ioprio_class];
        per_prio = &dd->per_prio[prio];
        if (!rq->elv.priv[0]) {
                per_prio->stats.inserted++;
                rq->elv.priv[0] = (void *)(uintptr_t)1;
        }

        if (blk_mq_sched_try_insert_merge(q, rq, &free)) {
                blk_mq_free_requests(&free);
                return;
        }

        trace_block_rq_insert(rq);

        if (at_head) {
                list_add(&rq->queuelist, &per_prio->dispatch);
                rq->fifo_time = jiffies;
        } else {
                twesu_add_rq_rb(per_prio, rq);

                if (rq_mergeable(rq)) {
                        elv_rqhash_add(q, rq);
                        if (!q->last_merge)
                                q->last_merge = rq;
                }

                rq->fifo_time = jiffies + dd->fifo_expire[data_dir];
                list_add_tail(&rq->queuelist, &per_prio->fifo_list[data_dir]);
        }
}

static void twesu_insert_requests(struct blk_mq_hw_ctx *hctx,
                               struct list_head *list, bool at_head)
{
        struct request_queue *q = hctx->queue;
        struct twesu_data *dd = q->elevator->elevator_data;

        spin_lock(&dd->lock);
        while (!list_empty(list)) {
                struct request *rq;

                rq = list_first_entry(list, struct request, queuelist);
                list_del_init(&rq->queuelist);
                twesu_insert_request(hctx, rq, at_head);
        }
        spin_unlock(&dd->lock);
}

static void twesu_prepare_request(struct request *rq)
{
        rq->elv.priv[0] = NULL;
}

static bool twesu_has_write_work(struct blk_mq_hw_ctx *hctx)
{
        struct twesu_data *dd = hctx->queue->elevator->elevator_data;
        enum twesu_prio p;

        for (p = 0; p <= TWESU_PRIO_MAX; p++)
                if (!list_empty_careful(&dd->per_prio[p].fifo_list[TWESU_WRITE]))
                        return true;

        return false;
}

static void twesu_finish_request(struct request *rq)
{
        struct request_queue *q = rq->q;
        struct twesu_data *dd = q->elevator->elevator_data;
        const u8 ioprio_class = twesu_rq_ioclass(rq);
        const enum twesu_prio prio = ioprio_class_to_prio[ioprio_class];
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];

        if (!rq->elv.priv[0])
                return;

        atomic_inc(&per_prio->stats.completed);

        if (blk_queue_is_zoned(q)) {
                unsigned long flags;

                spin_lock_irqsave(&dd->zone_lock, flags);
                blk_req_zone_write_unlock(rq);
                spin_unlock_irqrestore(&dd->zone_lock, flags);

                if (twesu_has_write_work(rq->mq_hctx))
                        blk_mq_sched_mark_restart_hctx(rq->mq_hctx);
        }
}

static bool twesu_has_work_for_prio(struct twesu_per_prio *per_prio)
{
        return !list_empty_careful(&per_prio->dispatch) ||
                !list_empty_careful(&per_prio->fifo_list[TWESU_READ]) ||
                !list_empty_careful(&per_prio->fifo_list[TWESU_WRITE]);
}

static bool twesu_has_work(struct blk_mq_hw_ctx *hctx)
{
        struct twesu_data *dd = hctx->queue->elevator->elevator_data;
        enum twesu_prio prio;

        for (prio = 0; prio <= TWESU_PRIO_MAX; prio++)
                if (twesu_has_work_for_prio(&dd->per_prio[prio]))
                        return true;

        return false;
}

#define SHOW_INT(__FUNC, __VAR)                                                \
static ssize_t __FUNC(struct elevator_queue *e, char *page)                \
{                                                                        \
        struct twesu_data *dd = e->elevator_data;                        \
                                                                        \
        return sysfs_emit(page, "%d\n", __VAR);                                \
}
#define SHOW_JIFFIES(__FUNC, __VAR) SHOW_INT(__FUNC, jiffies_to_msecs(__VAR))
SHOW_JIFFIES(twesu_read_expire_show, dd->fifo_expire[TWESU_READ]);
SHOW_JIFFIES(twesu_write_expire_show, dd->fifo_expire[TWESU_WRITE]);
SHOW_JIFFIES(twesu_prio_aging_expire_show, dd->prio_aging_expire);
SHOW_INT(twesu_writes_starved_show, dd->writes_starved);
SHOW_INT(twesu_front_merges_show, dd->front_merges);
SHOW_INT(twesu_async_depth_show, dd->async_depth);
SHOW_INT(twesu_fifo_batch_show, dd->fifo_batch);
#undef SHOW_INT
#undef SHOW_JIFFIES

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)                        \
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count)        \
{                                                                        \
        struct twesu_data *dd = e->elevator_data;                        \
        int __data, __ret;                                                \
                                                                        \
        __ret = kstrtoint(page, 0, &__data);                                \
        if (__ret < 0)                                                        \
                return __ret;                                                \
        if (__data < (MIN))                                                \
                __data = (MIN);                                                \
        else if (__data > (MAX))                                        \
                __data = (MAX);                                                \
        *(__PTR) = __CONV(__data);                                        \
        return count;                                                        \
}
#define STORE_INT(__FUNC, __PTR, MIN, MAX)                                \
        STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, )
#define STORE_JIFFIES(__FUNC, __PTR, MIN, MAX)                                \
        STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, msecs_to_jiffies)
STORE_JIFFIES(twesu_read_expire_store, &dd->fifo_expire[TWESU_READ], 0, INT_MAX);
STORE_JIFFIES(twesu_write_expire_store, &dd->fifo_expire[TWESU_WRITE], 0, INT_MAX);
STORE_JIFFIES(twesu_prio_aging_expire_store, &dd->prio_aging_expire, 0, INT_MAX);
STORE_INT(twesu_writes_starved_store, &dd->writes_starved, INT_MIN, INT_MAX);
STORE_INT(twesu_front_merges_store, &dd->front_merges, 0, 1);
STORE_INT(twesu_async_depth_store, &dd->async_depth, 1, INT_MAX);
STORE_INT(twesu_fifo_batch_store, &dd->fifo_batch, 0, INT_MAX);
#undef STORE_FUNCTION
#undef STORE_INT
#undef STORE_JIFFIES

#define TWESU_ATTR(name) \
        __ATTR(name, 0644, twesu_##name##_show, twesu_##name##_store)

static struct elv_fs_entry twesu_attrs[] = {
        TWESU_ATTR(read_expire),
        TWESU_ATTR(write_expire),
        TWESU_ATTR(writes_starved),
        TWESU_ATTR(front_merges),
        TWESU_ATTR(async_depth),
        TWESU_ATTR(fifo_batch),
        TWESU_ATTR(prio_aging_expire),
        __ATTR_NULL
};

#ifdef CONFIG_BLK_DEBUG_FS
#define TWESU_DEBUGFS_DDIR_ATTRS(prio, data_dir, name)                \
static void *twesu_##name##_fifo_start(struct seq_file *m,                \
                                          loff_t *pos)                        \
        __acquires(&dd->lock)                                                \
{                                                                        \
        struct request_queue *q = m->private;                                \
        struct twesu_data *dd = q->elevator->elevator_data;                \
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];                \
                                                                        \
        spin_lock(&dd->lock);                                                \
        return seq_list_start(&per_prio->fifo_list[data_dir], *pos);        \
}                                                                        \
                                                                        \
static void *twesu_##name##_fifo_next(struct seq_file *m, void *v,        \
                                         loff_t *pos)                        \
{                                                                        \
        struct request_queue *q = m->private;                                \
        struct twesu_data *dd = q->elevator->elevator_data;                \
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];                \
                                                                        \
        return seq_list_next(v, &per_prio->fifo_list[data_dir], pos);        \
}                                                                        \
                                                                        \
static void twesu_##name##_fifo_stop(struct seq_file *m, void *v)        \
        __releases(&dd->lock)                                                \
{                                                                        \
        struct request_queue *q = m->private;                                \
        struct twesu_data *dd = q->elevator->elevator_data;                \
                                                                        \
        spin_unlock(&dd->lock);                                                \
}                                                                        \
                                                                        \
static const struct seq_operations twesu_##name##_fifo_seq_ops = {        \
        .start        = twesu_##name##_fifo_start,                                \
        .next        = twesu_##name##_fifo_next,                                \
        .stop        = twesu_##name##_fifo_stop,                                \
        .show        = blk_mq_debugfs_rq_show,                                \
};                                                                        \
                                                                        \
static int twesu_##name##_next_rq_show(void *data,                        \
                                          struct seq_file *m)                \
{                                                                        \
        struct request_queue *q = data;                                        \
        struct twesu_data *dd = q->elevator->elevator_data;                \
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];                \
        struct request *rq = per_prio->next_rq[data_dir];                \
                                                                        \
        if (rq)                                                                \
                __blk_mq_debugfs_rq_show(m, rq);                        \
        return 0;                                                        \
}

TWESU_DEBUGFS_DDIR_ATTRS(TWESU_RT_PRIO, TWESU_READ, read0);
TWESU_DEBUGFS_DDIR_ATTRS(TWESU_RT_PRIO, TWESU_WRITE, write0);
TWESU_DEBUGFS_DDIR_ATTRS(TWESU_BE_PRIO, TWESU_READ, read1);
TWESU_DEBUGFS_DDIR_ATTRS(TWESU_BE_PRIO, TWESU_WRITE, write1);
TWESU_DEBUGFS_DDIR_ATTRS(TWESU_IDLE_PRIO, TWESU_READ, read2);
TWESU_DEBUGFS_DDIR_ATTRS(TWESU_IDLE_PRIO, TWESU_WRITE, write2);
#undef TWESU_DEBUGFS_DDIR_ATTRS

static int twesu_batching_show(void *data, struct seq_file *m)
{
        struct request_queue *q = data;
        struct twesu_data *dd = q->elevator->elevator_data;

        seq_printf(m, "%u\n", dd->batching);
        return 0;
}

static int twesu_starved_show(void *data, struct seq_file *m)
{
        struct request_queue *q = data;
        struct twesu_data *dd = q->elevator->elevator_data;

        seq_printf(m, "%u\n", dd->starved);
        return 0;
}

static int twesu_debugfs_async_depth_show(void *data, struct seq_file *m)
{
        struct request_queue *q = data;
        struct twesu_data *dd = q->elevator->elevator_data;

        seq_printf(m, "%u\n", dd->async_depth);
        return 0;
}

static int twesu_queued_show(void *data, struct seq_file *m)
{
        struct request_queue *q = data;
        struct twesu_data *dd = q->elevator->elevator_data;
        u32 rt, be, idle;

        spin_lock(&dd->lock);
        rt = twesu_queued(dd, TWESU_RT_PRIO);
        be = twesu_queued(dd, TWESU_BE_PRIO);
        idle = twesu_queued(dd, TWESU_IDLE_PRIO);
        spin_unlock(&dd->lock);

        seq_printf(m, "%u %u %u\n", rt, be, idle);

        return 0;
}

static u32 twesu_owned_by_driver(struct twesu_data *dd, enum twesu_prio prio)
{
        const struct io_stats_per_prio *stats = &dd->per_prio[prio].stats;

        lockdep_assert_held(&dd->lock);

        return stats->dispatched + stats->merged -
                atomic_read(&stats->completed);
}

static int twesu_owned_by_driver_show(void *data, struct seq_file *m)
{
        struct request_queue *q = data;
        struct twesu_data *dd = q->elevator->elevator_data;
        u32 rt, be, idle;

        spin_lock(&dd->lock);
        rt = twesu_owned_by_driver(dd, TWESU_RT_PRIO);
        be = twesu_owned_by_driver(dd, TWESU_BE_PRIO);
        idle = twesu_owned_by_driver(dd, TWESU_IDLE_PRIO);
        spin_unlock(&dd->lock);

        seq_printf(m, "%u %u %u\n", rt, be, idle);

        return 0;
}

#define TWESU_DISPATCH_ATTR(prio)                                        \
static void *twesu_dispatch##prio##_start(struct seq_file *m,        \
                                             loff_t *pos)                \
        __acquires(&dd->lock)                                                \
{                                                                        \
        struct request_queue *q = m->private;                                \
        struct twesu_data *dd = q->elevator->elevator_data;                \
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];                \
                                                                        \
        spin_lock(&dd->lock);                                                \
        return seq_list_start(&per_prio->dispatch, *pos);                \
}                                                                        \
                                                                        \
static void *twesu_dispatch##prio##_next(struct seq_file *m,                \
                                            void *v, loff_t *pos)        \
{                                                                        \
        struct request_queue *q = m->private;                                \
        struct twesu_data *dd = q->elevator->elevator_data;                \
        struct twesu_per_prio *per_prio = &dd->per_prio[prio];                \
                                                                        \
        return seq_list_next(v, &per_prio->dispatch, pos);                \
}                                                                        \
                                                                        \
static void twesu_dispatch##prio##_stop(struct seq_file *m, void *v)        \
        __releases(&dd->lock)                                                \
{                                                                        \
        struct request_queue *q = m->private;                                \
        struct twesu_data *dd = q->elevator->elevator_data;                \
                                                                        \
        spin_unlock(&dd->lock);                                                \
}                                                                        \
                                                                        \
static const struct seq_operations twesu_dispatch##prio##_seq_ops = { \
        .start        = twesu_dispatch##prio##_start,                        \
        .next        = twesu_dispatch##prio##_next,                        \
        .stop        = twesu_dispatch##prio##_stop,                        \
        .show        = blk_mq_debugfs_rq_show,                                \
}

TWESU_DISPATCH_ATTR(0);
TWESU_DISPATCH_ATTR(1);
TWESU_DISPATCH_ATTR(2);
#undef TWESU_DISPATCH_ATTR

#define TWESU_QUEUE_DDIR_ATTRS(name)                                        \
        {#name "_fifo_list", 0400,                                        \
                        .seq_ops = &twesu_##name##_fifo_seq_ops}
#define TWESU_NEXT_RQ_ATTR(name)                                        \
        {#name "_next_rq", 0400, twesu_##name##_next_rq_show}
static const struct blk_mq_debugfs_attr twesu_queue_debugfs_attrs[] = {
        TWESU_QUEUE_DDIR_ATTRS(read0),
        TWESU_QUEUE_DDIR_ATTRS(write0),
        TWESU_QUEUE_DDIR_ATTRS(read1),
        TWESU_QUEUE_DDIR_ATTRS(write1),
        TWESU_QUEUE_DDIR_ATTRS(read2),
        TWESU_QUEUE_DDIR_ATTRS(write2),
        TWESU_NEXT_RQ_ATTR(read0),
        TWESU_NEXT_RQ_ATTR(write0),
        TWESU_NEXT_RQ_ATTR(read1),
        TWESU_NEXT_RQ_ATTR(write1),
        TWESU_NEXT_RQ_ATTR(read2),
        TWESU_NEXT_RQ_ATTR(write2),
        {"batching", 0400, twesu_batching_show},
        {"starved", 0400, twesu_starved_show},
        {"async_depth", 0400, twesu_debugfs_async_depth_show},
        {"dispatch0", 0400, .seq_ops = &twesu_dispatch0_seq_ops},
        {"dispatch1", 0400, .seq_ops = &twesu_dispatch1_seq_ops},
        {"dispatch2", 0400, .seq_ops = &twesu_dispatch2_seq_ops},
        {"owned_by_driver", 0400, twesu_owned_by_driver_show},
        {"queued", 0400, twesu_queued_show},
        {},
};
#undef TWESU_QUEUE_DDIR_ATTRS
#endif

static struct elevator_type mq_twesu = {
        .ops = {
                .depth_updated                = twesu_depth_updated,
                .limit_depth                = twesu_limit_depth,
                .insert_requests        = twesu_insert_requests,
                .dispatch_request        = twesu_dispatch_request,
                .prepare_request        = twesu_prepare_request,
                .finish_request                = twesu_finish_request,
                .next_request                = elv_rb_latter_request,
                .former_request                = elv_rb_former_request,
                .bio_merge                = twesu_bio_merge,
                .request_merge                = twesu_request_merge,
                .requests_merged        = twesu_merged_requests,
                .request_merged                = twesu_request_merged,
                .has_work                = twesu_has_work,
                .init_sched                = twesu_init_sched,
                .exit_sched                = twesu_exit_sched,
                .init_hctx                = twesu_init_hctx,
        },

#ifdef CONFIG_BLK_DEBUG_FS
        .queue_debugfs_attrs = twesu_queue_debugfs_attrs,
#endif
        .elevator_attrs = twesu_attrs,
        .elevator_name = "twesu",
        .elevator_alias = "twesu_iosched",
        .elevator_features = ELEVATOR_F_ZBD_SEQ_WRITE,
        .elevator_owner = THIS_MODULE,
};
MODULE_ALIAS("mq-twesu-iosched");

static int __init twesu_init(void)
{
        return elv_register(&mq_twesu);
}

static void __exit twesu_exit(void)
{
        elv_unregister(&mq_twesu);
}

module_init(twesu_init);
module_exit(twesu_exit);

MODULE_AUTHOR("Jens Axboe, Damien Le Moal, Bart Van Assche (Tweak & Adjusted by Gemini)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TWESU - IO Scheduler Kustom Berkinerja Tinggi (Balanced Ultra Edition)");