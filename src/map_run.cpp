
#include "map_run.hpp"

#include "task_thread.hpp"
#include <iostream>


int main()
{

  task_thread::ConcurrentQueue<std::unique_ptr<int>> cqi{};
  cqi.push(std::make_unique<int>(5));
  auto i6 = std::make_unique<int>(6);
  cqi.push(std::move(i6));
  std::unique_ptr<int> pi5;
  cqi.try_pop(pi5);
  std::cout << *pi5 << " " << *cqi.pop() << std::endl;

  struct Constructed
  {
    int i_;

    Constructed(int i) : i_{i}
    {}
  };
  task_thread::ConcurrentQueue<Constructed> cqc{};
  Constructed c3{3};
  cqc.push(c3);
  cqc.push(Constructed{7});
  Constructed c3a{-3};
  cqc.try_pop(c3a);
  std::cout << c3a.i_ << " " << cqc.pop().i_ << std::endl;


  task_thread::ConcurrentQueue<int> out_q{}; // The output queue must have a longer life than the TaskThread
  auto work = std::make_unique<int>(5);
  task_thread::TaskThread<int> tti{std::move(work)};
  tti.push([&out_q](int &i)
           {
             i += 2;
             out_q.push(i);
           });
  tti.push([&out_q](int &i)
           {
             i += 3;
             out_q.push(i);
           });
  std::cout << out_q.pop() << " " << out_q.pop() << std::endl;

//  camsim::map_global(0.1, 0.3, 01.0, 0.5);
  camsim::map_global_thread(0.1, 0.3, 01.0, 0.5);
}

