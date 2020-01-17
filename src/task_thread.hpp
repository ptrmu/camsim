
#ifndef _TASK_THREAD_HPP
#define _TASK_THREAD_HPP

#include <atomic>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace task_thread
{
  // Items are moved into the queue. In other words, The queue takes
  // ownership of the items placed in it.
  template<typename TItem>
  class ConcurrentQueue
  {
    std::queue<TItem> q_{};
    std::mutex m_{};
    std::condition_variable cv_{};

  public:
    void push(TItem item)
    {
      std::unique_lock<std::mutex> lock{m_};
      q_.push(std::move(item));
      lock.unlock();
      cv_.notify_one();
    }

    bool empty()
    {
      std::unique_lock<std::mutex> lock{m_};
      return q_.empty();
    }

    bool try_pop(TItem &popped_item)
    {
      std::unique_lock<std::mutex> lock{m_};
      if (q_.empty()) {
        return false;
      }

      popped_item = std::move(q_.front());
      q_.pop();
      return true;
    }

    void wait_for()
    {
      std::unique_lock<std::mutex> lock{m_};
      cv_.wait_for(lock, std::chrono::milliseconds(250));
    }

    TItem wait_and_pop()
    {
      std::unique_lock<std::mutex> lock{m_};
      while (q_.empty()) {
        cv_.wait_for(lock, std::chrono::milliseconds(250));
      }

      TItem popped_item{std::move(q_.front())};
      q_.pop();
      return popped_item;
    }

    void notify_one()
    {
      cv_.notify_one();
    }

    std::size_t size()
    {
      std::unique_lock<std::mutex> lock{m_};
      return q_.size();
    }
  };

  // TaskThread is instantiated with a work object. The functors in the queue
  // contain code that operates on the work object. The functors are executed
  // on a thread. A convenient way to return results from a functor calculation
  // is with another ConcurrentQueue.
  template<class TWork>
  class TaskThread
  {
    ConcurrentQueue<std::function<void(TWork &)>> q_{};
    std::unique_ptr<TWork> work_;
    std::thread thread_;
    int abort_{0};

    static void run(TaskThread *tt)
    {
      while (true) {
        std::function<void(TWork &)> task{};
        bool run_task = tt->q_.try_pop(task);
        if (tt->abort_) {
          break;
        }
        if (run_task) {
          task(*tt->work_);
        }
        if (tt->q_.empty()) {
          tt->q_.wait_for();
        }
      }
    }

  public:
    TaskThread(std::unique_ptr<TWork> work) :
      work_{std::move(work)}, thread_{run, this}
    {}

    ~TaskThread()
    {
      abort_ = 1;
      q_.notify_one();
      thread_.join();
    }

    void abort()
    {
      abort_ = 1;
      q_.notify_one();
    }

    void push(const std::function<void(TWork &)> &task)
    {
      q_.push(task);
    }

    bool empty()
    {
      return q_.empty();
    }

    std::size_t tasks_queued()
    {
      return q_.size();
    }

    void wait_until_empty()
    {
      while (!empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  };
}
#endif //_TASK_THREAD_HPP
