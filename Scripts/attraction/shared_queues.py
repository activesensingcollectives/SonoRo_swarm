import queue


def _queue_clear(self):
    """Thread-safely clear all items from the queue."""
    with self.mutex:
        self.queue.clear()
        if hasattr(self, "all_tasks_done"):
            self.all_tasks_done.notify_all()
        if hasattr(self, "not_full"):
            self.not_full.notify_all()


if not hasattr(queue.Queue, "clear"):
    queue.Queue.clear = _queue_clear


def clear_queue(q):
    """Thread-safely clear all items from a Queue/LifoQueue."""
    if hasattr(q, "clear"):
        q.clear()
    else:
        _queue_clear(q)


# Global queues for sharing data across threads
angle_queue = queue.LifoQueue()
level_queue = queue.LifoQueue()

angle_queue_led = queue.LifoQueue()
level_queue_led = queue.LifoQueue()
