#include <ros/ros.h>
#include <queue>
#include "./task_queue.h"

class RISE_Assembler_Controller
{
  private:
    std::queue<Task> taskQueue;
    
  public:
    RISE_Assembler_Controller();
    int RISE_Assembler_Controller::parseTask(char*);
};

RISE_Assembler_Controller::RISE_Assembler_Controller()
{
    taskQueue.empty();
}

int RISE_Assembler_Controller::parseTask(char *file_path)
{
    
}