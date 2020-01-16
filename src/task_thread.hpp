
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
  template<typename TItem>
  class ConcurrentQueue
  {
    std::queue<TItem> q_;
    std::mutex m_;
    std::condition_variable cv_;

  public:
    void push(const TItem &item)
    {
      std::unique_lock<std::mutex> lock{m_};
      q_.push(item);
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

      popped_item = q_.front();
      q_.pop();
      return true;
    }

    void wait_for()
    {
      std::unique_lock<std::mutex> lock{m_};
      cv_.wait_for(lock, std::chrono::milliseconds(250));
    }

    void notify_one()
    {
      cv_.notify_one();
    }
  };


  template<class TWork>
  class TaskThread
  {
    ConcurrentQueue<std::function<void(TWork &)>> q_{};
    std::unique_ptr<TWork> work_;
    std::thread thread_;
    std::atomic_flag run_flag_{true};

    static void run(TaskThread *tt)
    {
      while (tt->run_flag_.test_and_set()) {
        std::function<void(TWork &)> task{};
        while (tt->q_.try_pop(task)) {
          task(*tt->work_);
        }
        tt->q_.wait_for();
      }
    }

  public:
    TaskThread(std::unique_ptr<TWork> &work) :
      work_{std::move(work)},
      thread_{run, this}
    {}

    ~TaskThread()
    {
      run_flag_.clear();
      q_.notify_one();
      thread_.join();
    }

    void abort()
    {
      run_flag_.clear();
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

    void wait_until_empty()
    {
      while (!empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  };
}
#endif //_TASK_THREAD_HPP
