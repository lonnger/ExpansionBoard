#pragma once
#include "FreeRTOS.h"
#include "main.h"
namespace Core
{
namespace Utils
{
namespace TaskManager
{
template <class T, size_t StackDepth>
class TaskManager
{
   public:
    TaskManager() = delete;
    TaskManager(const char *const taskName_, UBaseType_t taskPriority_) : taskName(taskName_), taskPriority(taskPriority_) {}

    TaskHandle_t getTaskHandle() const { return taskHandle; }

    void create()
    {
        BaseType_t result = xTaskCreateStatic(taskFunc, taskName, StackDepth, this, taskPriority, taskStack, &taskTCB);
        configASSERT(result == pdPASS);
    }

   private:
    static void taskFunc(void *param)
    {
        T *task = static_cast<T *>(param);
        task->run();
    }
    const char taskName[100];
    static StackType_t taskStack[StackDepth];
    StaticTask_t taskTCB;
    TaskHandle_t taskHandle;
    UBaseType_t taskPriority;
};

}  // namespace TaskManager
}  // namespace Utils
}  // namespace Core