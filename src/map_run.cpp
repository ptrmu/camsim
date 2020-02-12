
#include "map_run.hpp"

#include "task_thread.hpp"

#include <iostream>


int main()
{
#if 0
  { // ConcurentQueue with int as argument
    task_thread::ConcurrentQueue<int> cqi{};
    cqi.push(2);
    auto i1 = 1;
    cqi.push(i1);
    int pi2;
    cqi.try_pop(pi2);
    std::cout << pi2 << " " << cqi.pop() << std::endl;
  }

  {
    // ConcurentQueue with unique ptr to int as payload.
    // This demonstrates how the queue manages ownership.
    task_thread::ConcurrentQueue<std::unique_ptr<int>> cqi{};
    cqi.push(std::make_unique<int>(5));
    auto i6 = std::make_unique<int>(6);
    cqi.push(std::move(i6));
    std::unique_ptr<int> pi5;
    cqi.try_pop(pi5);
    std::cout << *pi5 << " " << *cqi.pop() << std::endl;
  }

  {
    // ConcurrentQueue with a struct/class as the payload.
    // This demonstrates that classes without default constructors
    // can be payloads.
    struct Constructed
    {
      int i_;

      Constructed(int i) : i_{i}
      {}

      Constructed() = delete;
    };
    task_thread::ConcurrentQueue<Constructed> cqc{};
    Constructed c3{3};
    cqc.push(c3);
    cqc.push(Constructed{7});
    Constructed c3a{-3};
    cqc.try_pop(c3a);
    std::cout << c3a.i_ << " " << cqc.pop().i_ << std::endl;
  }

  {
    // TaskThread demonstrating how to use a ConcurrentQueue
    // to return results from task.
    task_thread::ConcurrentQueue<int> out_q{}; // The output queue must have a longer life than the TaskThread
    auto work = std::make_unique<int>(3);
    task_thread::TaskThread<int> tti{std::move(work)};
    tti.push([&out_q](int &i)
             {
               i += 2;
               out_q.push(i);
             });
    tti.push(std::function<void(int &)>{[&out_q](int &i)
                                        {
                                          i += 3;
                                          out_q.push(i);
                                        }});
    std::cout << out_q.pop() << " " << out_q.pop() << std::endl;
  }

  {
    // TaskThread demonstrating how to use promise/future pairs
    // to return results from a task.
    auto work = std::make_unique<int>(14);
    task_thread::TaskThread<int> tti{std::move(work)};
    std::promise<int> p1;
    auto f1 = p1.get_future();
    tti.push([p1 = std::move(p1)](int &i) mutable
             {
               i += 2;
               p1.set_value(i);
             });
    std::promise<int> p2;
    auto f2 = p2.get_future();
    tti.push([p2 = std::move(p2)](int &i) mutable
             {
               i += 3;
               p2.set_value(i);;
             });
    std::cout << f1.get() << " " << f2.get() << std::endl;
  }

  {
    // TaskThread demonstrating how to pass a functor that hos no
    // default constructor and is movable but not copyable.
    task_thread::ConcurrentQueue<int> out_q{}; // The output queue must have a longer life than the TaskThread
    auto work = std::make_unique<int>(21);
    task_thread::TaskThread<int> tti{std::move(work)};
    class ATask
    {
      task_thread::ConcurrentQueue<int> &out_q_;
      std::unique_ptr<int> pi;

    public:
      ATask(int i_inc, task_thread::ConcurrentQueue<int> &out_q) : out_q_{out_q}, pi{std::make_unique<int>(i_inc)}
      {}

      ATask() = delete;

      void operator()(int &i)
      {
        i = i + *pi;
        out_q_.push(i);
      }
    };
    ATask atask{4, out_q};
    tti.push(std::move(atask)); // atask has been moved and can not used again
    tti.push(ATask{3, out_q});
    std::cout << out_q.pop() << " " << out_q.pop() << std::endl;
  }

  camsim::map_global_thread(0.1, 0.3, 01.0, 0.5);
#else
  camsim::map_global(0.1, 0.3, 01.0, 0.5);
#endif
}

