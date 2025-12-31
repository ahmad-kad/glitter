#!/usr/bin/env python3
"""
Asynchronous I/O Pipeline for Real-Time Processing
==================================================

Non-blocking I/O operations with producer-consumer queues to prevent
ROS communication from blocking the main processing thread.
"""

import asyncio
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Optional, Callable, Dict, List
from collections import deque
import queue


class AsyncIOPipeline:
    """
    Asynchronous I/O pipeline with producer-consumer pattern.

    Prevents blocking ROS operations from halting real-time processing.
    """

    def __init__(self, max_queue_size: int = 10, num_workers: int = 2):
        # Threading components
        self.loop = None
        self.executor = ThreadPoolExecutor(max_workers=num_workers, thread_name_prefix="async_io")
        self.processing_thread = None

        # Queues for async communication
        self.input_queue = asyncio.Queue(maxsize=max_queue_size)
        self.output_queue = asyncio.Queue(maxsize=max_queue_size)
        self.processing_queue = asyncio.Queue(maxsize=max_queue_size)

        # Synchronization
        self.running = False
        self.lock = threading.RLock()

        # Statistics
        self.stats = {
            'messages_received': 0,
            'messages_processed': 0,
            'messages_sent': 0,
            'queue_full_events': 0,
            'processing_errors': 0,
            'avg_processing_time': 0.0,
            'max_queue_depth': 0
        }

        # Callbacks
        self.input_callback: Optional[Callable] = None
        self.output_callback: Optional[Callable] = None

    def start(self):
        """Start the asynchronous pipeline"""
        with self.lock:
            if self.running:
                return

            self.running = True

            # Start asyncio event loop in separate thread
            def run_event_loop():
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
                self.loop.run_until_complete(self._run_pipeline())

            self.processing_thread = threading.Thread(target=run_event_loop, daemon=True)
            self.processing_thread.start()

    def stop(self):
        """Stop the asynchronous pipeline"""
        with self.lock:
            self.running = False

            if self.loop:
                self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self._shutdown()))
                self.processing_thread.join(timeout=2.0)

            self.executor.shutdown(wait=True)

    async def _shutdown(self):
        """Async shutdown procedure"""
        # Cancel all pending tasks
        for task in asyncio.all_tasks(self.loop):
            if not task.done():
                task.cancel()

    async def _run_pipeline(self):
        """Main async pipeline loop"""
        try:
            while self.running:
                try:
                    # Process input -> processing -> output pipeline
                    await asyncio.gather(
                        self._process_inputs(),
                        self._process_outputs(),
                        return_exceptions=True
                    )
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    print(f"Pipeline error: {e}")
                    await asyncio.sleep(0.1)

        except Exception as e:
            print(f"Pipeline shutdown error: {e}")

    async def _process_inputs(self):
        """Process input queue"""
        try:
            while self.running:
                # Wait for input with timeout
                try:
                    input_data = await asyncio.wait_for(
                        self.input_queue.get(), timeout=0.1
                    )
                except asyncio.TimeoutError:
                    continue

                # Submit to thread pool for processing
                self.stats['messages_received'] += 1

                # Process in thread pool to avoid blocking
                future = self.loop.run_in_executor(
                    self.executor, self._process_input_data, input_data
                )

                # Schedule output handling
                future.add_done_callback(self._handle_processing_result)

                # Update queue depth stats
                self.stats['max_queue_depth'] = max(
                    self.stats['max_queue_depth'], self.input_queue.qsize()
                )

        except Exception as e:
            print(f"Input processing error: {e}")

    def _process_input_data(self, input_data) -> Any:
        """Process input data in thread pool"""
        start_time = time.time()

        try:
            # Call user-defined processing callback
            if self.input_callback:
                result = self.input_callback(input_data)
            else:
                result = input_data  # Passthrough if no callback

            # Update processing stats
            processing_time = time.time() - start_time
            self.stats['messages_processed'] += 1

            # Update average processing time
            total_processed = self.stats['messages_processed']
            self.stats['avg_processing_time'] = (
                (self.stats['avg_processing_time'] * (total_processed - 1)) +
                processing_time
            ) / total_processed

            return result

        except Exception as e:
            self.stats['processing_errors'] += 1
            print(f"Input processing failed: {e}")
            return None

    def _handle_processing_result(self, future):
        """Handle completed processing result"""
        try:
            result = future.result()
            if result is not None:
                # Schedule output in event loop
                self.loop.call_soon_threadsafe(
                    lambda: asyncio.create_task(self._queue_output(result))
                )
        except Exception as e:
            print(f"Processing result handling failed: {e}")

    async def _queue_output(self, result):
        """Queue result for output"""
        try:
            await self.output_queue.put(result)
        except asyncio.QueueFull:
            self.stats['queue_full_events'] += 1
            print("Output queue full - dropping result")

    async def _process_outputs(self):
        """Process output queue"""
        try:
            while self.running:
                # Wait for output with timeout
                try:
                    output_data = await asyncio.wait_for(
                        self.output_queue.get(), timeout=0.1
                    )
                except asyncio.TimeoutError:
                    continue

                # Call output callback
                if self.output_callback:
                    try:
                        self.output_callback(output_data)
                        self.stats['messages_sent'] += 1
                    except Exception as e:
                        print(f"Output callback failed: {e}")
                else:
                    self.stats['messages_sent'] += 1

        except Exception as e:
            print(f"Output processing error: {e}")

    def submit_input(self, data: Any) -> bool:
        """Submit data to input queue (non-blocking)"""
        if not self.running or not self.loop:
            return False

        try:
            # Try to put in queue without blocking
            future = asyncio.run_coroutine_threadsafe(
                self.input_queue.put(data), self.loop
            )

            # Check if it succeeded (with small timeout)
            future.result(timeout=0.01)
            return True

        except asyncio.QueueFull:
            self.stats['queue_full_events'] += 1
            return False
        except Exception:
            return False

    def set_input_callback(self, callback: Callable[[Any], Any]):
        """Set callback for processing input data"""
        self.input_callback = callback

    def set_output_callback(self, callback: Callable[[Any], None]):
        """Set callback for handling output data"""
        self.output_callback = callback

    def get_stats(self) -> Dict[str, Any]:
        """Get pipeline statistics"""
        with self.lock:
            return self.stats.copy()

    def reset_stats(self):
        """Reset statistics counters"""
        with self.lock:
            for key in self.stats:
                if key.startswith(('avg_', 'max_')):
                    continue  # Keep averages and maximums
                self.stats[key] = 0


class ROSAsyncBridge:
    """
    Bridge between ROS callbacks and async pipeline.

    Enables non-blocking ROS message handling.
    """

    def __init__(self, pipeline: AsyncIOPipeline):
        self.pipeline = pipeline
        self.message_buffer = deque(maxlen=100)  # Buffer for ROS messages

    def ros_callback_bridge(self, ros_msg) -> None:
        """
        ROS callback that submits to async pipeline.

        Use this as your ROS subscription callback for non-blocking operation.
        """
        # Add timestamp for latency tracking
        enhanced_msg = {
            'ros_msg': ros_msg,
            'receive_time': time.time(),
            'topic': getattr(ros_msg, '_topic', 'unknown')
        }

        # Submit to async pipeline
        success = self.pipeline.submit_input(enhanced_msg)

        if not success:
            print(f"ROS message dropped - pipeline full (topic: {enhanced_msg['topic']})")

    def get_latency_stats(self) -> Dict[str, float]:
        """Get message latency statistics"""
        if not self.message_buffer:
            return {'avg_latency': 0.0, 'max_latency': 0.0}

        latencies = []
        current_time = time.time()

        for msg in self.message_buffer:
            if 'receive_time' in msg:
                latency = current_time - msg['receive_time']
                latencies.append(latency)

        return {
            'avg_latency': sum(latencies) / len(latencies) if latencies else 0.0,
            'max_latency': max(latencies) if latencies else 0.0,
            'buffered_messages': len(self.message_buffer)
        }


class AsyncProcessingBenchmark:
    """Benchmark async pipeline performance"""

    def __init__(self):
        self.pipeline = AsyncIOPipeline(max_queue_size=50, num_workers=4)
        self.bridge = ROSAsyncBridge(self.pipeline)

        # Configure pipeline
        self.pipeline.set_input_callback(self._simulate_processing)
        self.pipeline.set_output_callback(self._handle_output)

        self.results = []

    def _simulate_processing(self, data):
        """Simulate processing work"""
        # Simulate variable processing time
        processing_time = 0.001 + (hash(str(data)) % 100) / 100000.0
        time.sleep(processing_time)

        return {
            'input': data,
            'processing_time': processing_time,
            'output_time': time.time()
        }

    def _handle_output(self, result):
        """Handle processed output"""
        self.results.append(result)

    def benchmark_throughput(self, num_messages: int = 1000) -> Dict[str, Any]:
        """Benchmark pipeline throughput"""
        print(f"Benchmarking async pipeline with {num_messages} messages...")

        self.pipeline.start()
        start_time = time.time()

        # Send messages
        for i in range(num_messages):
            test_msg = {'id': i, 'data': f'message_{i}'}
            self.bridge.ros_callback_bridge(test_msg)

            # Small delay to simulate real ROS timing
            time.sleep(0.001)

        # Wait for processing to complete
        timeout = 10.0
        wait_start = time.time()
        while len(self.results) < num_messages and (time.time() - wait_start) < timeout:
            time.sleep(0.1)

        end_time = time.time()
        total_time = end_time - start_time

        # Calculate statistics
        messages_processed = len(self.results)
        throughput = messages_processed / total_time if total_time > 0 else 0

        processing_times = [r['processing_time'] for r in self.results[:100]]  # First 100

        stats = self.pipeline.get_stats()

        results = {
            'total_time': total_time,
            'messages_sent': num_messages,
            'messages_processed': messages_processed,
            'throughput_msg_per_sec': throughput,
            'processing_times_avg': sum(processing_times) / len(processing_times) if processing_times else 0,
            'pipeline_stats': stats
        }

        self.pipeline.stop()

        print(".3f")
        print(".1f")
        print(".3f")

        return results

    def benchmark_latency(self, num_messages: int = 100) -> Dict[str, Any]:
        """Benchmark end-to-end latency"""
        print(f"Benchmarking latency with {num_messages} messages...")

        self.pipeline.start()
        latencies = []

        for i in range(num_messages):
            send_time = time.time()

            test_msg = {'id': i, 'send_time': send_time}
            self.bridge.ros_callback_bridge(test_msg)

            # Wait for result (with timeout)
            timeout = 1.0
            wait_start = time.time()
            while not self.results and (time.time() - wait_start) < timeout:
                time.sleep(0.001)

            if self.results:
                result = self.results.pop(0)
                receive_time = time.time()
                latency = receive_time - send_time
                latencies.append(latency)

        self.pipeline.stop()

        if latencies:
            return {
                'avg_latency': sum(latencies) / len(latencies),
                'min_latency': min(latencies),
                'max_latency': max(latencies),
                'samples': len(latencies)
            }
        else:
            return {'error': 'No messages processed'}


if __name__ == "__main__":
    # Run benchmark
    benchmark = AsyncProcessingBenchmark()

    print("=== Async I/O Pipeline Benchmark ===")

    throughput_results = benchmark.benchmark_throughput(500)
    print(f"Throughput: {throughput_results['throughput_msg_per_sec']:.1f} msg/sec")

    # Reset for latency test
    benchmark.results = []
    benchmark.pipeline.reset_stats()

    latency_results = benchmark.benchmark_latency(50)
    if 'avg_latency' in latency_results:
        print(".1f")
        print(".1f")

    print("\nBenchmark completed!")

