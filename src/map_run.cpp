
#include "map_run.hpp"

#include "task_thread.hpp"
#include <iostream>


int main()
{

  task_thread::ConcurrentQueue<std::unique_ptr<int>> cq{};
  cq.push(std::make_unique<int>(5));
  auto i6 = std::make_unique<int>(6);
  cq.push(std::move(i6));
  std::unique_ptr<int> pi5;
  std::unique_ptr<int> pi6;
  cq.try_pop(pi5);
  cq.try_pop(pi6);
  std::cout << *pi5 << " " << *pi6 << std::endl;


  task_thread::ConcurrentQueue<int> out_q{}; // Make output queue have longer life than TaskThread
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
  std::cout << "queued tasks " << tti.tasks_queued() << std::endl;
  std::cout << "queued output " << out_q.size() << std::endl;
  std::cout << "first output " << out_q.wait_and_pop() << std::endl;
  std::cout << "second output " << out_q.wait_and_pop() << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "queued tasks " << tti.tasks_queued() << std::endl;

//  camsim::map_global(0.1, 0.3, 01.0, 0.5);
  camsim::map_global_thread(0.1, 0.3, 01.0, 0.5);
}

