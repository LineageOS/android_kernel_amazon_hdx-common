#include "msm_ispif_frame_event.h"
#include "msm_ispif.h"
#include "msm.h"
#include <asm/atomic.h>

struct workqueue_struct *frame_event_nrt_wq __read_mostly;
// In microseconds
static int timediff(const struct timespec *t1, const struct timespec *t0)
{
        return
                ((t1->tv_sec - t0->tv_sec) * 1000000) +
                ((t1->tv_nsec - t0->tv_nsec) / 1000);
}


void frame_event_workqueue_create(void)
{
        frame_event_nrt_wq = alloc_workqueue("frame_event_nrt_wq",WQ_NON_REENTRANT|WQ_HIGHPRI,1);
        WARN_ON(!frame_event_nrt_wq);
}

void frame_event_workqueue_destroy(void)
{
        destroy_workqueue(frame_event_nrt_wq);
}

uint8_t frame_event_queue_insert_idx(struct frame_event_mgr* fem)
{
	fem->fe_writer++;
	if(fem->fe_writer >= MAX_QUEUE_SIZE)
	fem->fe_writer = 0;

	return fem->fe_writer;
}

uint8_t frame_event_queue_pop_idx(struct frame_event_mgr* fem)
{
	fem->fe_reader++;
	if(fem->fe_reader >= MAX_QUEUE_SIZE)
	fem->fe_reader = 0;

	return fem->fe_reader;
}

void frame_event_manager_reset(struct frame_event_mgr *fem)
{
	pr_debug("%s", __func__);
	flush_work_sync(&fem->async_work);

	/* acquire mutex to make sure that we don't update the atomic variable
	 * while the worker function is executing */
	mutex_lock(&fem->sync_mutex);
	atomic_set(&fem->outstanding,0);
	fem->fe_writer = 0;
	fem->fe_reader = 0;
	fem->qf_count = 0;
	mutex_unlock(&fem->sync_mutex);

}

void dispatch_async_frame_event(struct ispif_device *ispif, enum msm_ispif_vfe_intf vfe_idx, enum msm_ispif_intftype interface, enum msm_ispif_frame_evt event)
{
	struct frame_event_mgr *fem =  (struct frame_event_mgr *)&ispif->fem;
	int evt_pending;
	evt_pending = atomic_read(&fem->outstanding);

	if(evt_pending <= MAX_QUEUE_SIZE)
	{
		uint8_t idx = frame_event_queue_insert_idx(fem);

		fem->queue[idx].vfe_idx = vfe_idx;
		fem->queue[idx].interface = interface;
		fem->queue[idx].event = event;
		getnstimeofday(&fem->queue[idx].timestamp);

		atomic_inc(&fem->outstanding);
		smp_mb__after_atomic_inc();

		//pr_debug("%c", event?'E':'S');
	}
	else
	{
		/* skip update as Queue is full, update QF count */
		fem->qf_count++;
		pr_debug("QF");
	}

	/* schedule a worker thread and make sensor subdev ioctl calls */
	queue_work(frame_event_nrt_wq, &fem->async_work);

}

void async_frame_event_do_work(struct work_struct *data)
{
	int evt_pending;
	int idx;
	struct frame_event_mgr *fem = container_of(data, struct frame_event_mgr, async_work);
	struct timespec t1,t2;
	ktime_t start, intermediate, delta;
	uint64_t time_spent;
	int rc;

	pr_debug("%s R%d W%d", __func__, fem->fe_reader, fem->fe_writer);

	if(fem->qf_count)
	{
		pr_err("%s QF[%d]",__func__, fem->qf_count);
		fem->qf_count = 0;

	}

	start = ktime_get();

	if(mutex_trylock(&fem->sync_mutex))
	{
		evt_pending = atomic_read(&fem->outstanding);
		while(evt_pending > 0)
		{
			idx = frame_event_queue_pop_idx(fem);
			/* notify sensor subdevs with  fem->queue[idx] */
			getnstimeofday(&t1);
			rc = msm_sensor_sd_ioctl_call(VIDIOC_SENSOR_FRAME_EVENT_HOOK, (void *)&fem->queue[idx]);
			if(rc)
			{
				pr_err("%s ioctl call failed %d", __func__, rc);
				break;
			}
			getnstimeofday(&t2);
			pr_debug("event[%s] R%d W%d took %d us", fem->queue[idx].event?"EOF":"SOF", fem->fe_reader, fem->fe_writer, timediff(&t2,&t1));

			atomic_dec(&fem->outstanding);
			evt_pending = atomic_read(&fem->outstanding);

			intermediate = ktime_get();
			delta = ktime_sub(intermediate, start);
			time_spent = ktime_to_ns(delta);

			if(unlikely(time_spent > 500000000LL))
			{
				pr_err("%s greater than 500ms spent in while loop %llu ns", __func__, time_spent);
				WARN(1,"camera: %s WQ running very long", __func__);
				/* now being nice to others */
				break;
			}

			if(unlikely(evt_pending < 0))
			{
				pr_err("%s evt_pending -ve [%d]", __func__, evt_pending);
				/* for safety */
				atomic_set(&fem->outstanding,0);
				break;
			}

		};
		mutex_unlock(&fem->sync_mutex);
	}
	else /* didn't get the mutex, we shouldn't get here */
	{
		pr_err("%s unexpected concurrency", __func__);
		return;
	}

}

