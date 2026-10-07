import queue

# Global queues for sharing data across threads
angle_queue = queue.LifoQueue()
level_queue = queue.LifoQueue()

angle_queue_led = queue.LifoQueue()
level_queue_led = queue.LifoQueue()
