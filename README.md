# What this is
These are my "tests" of FreeRTOS running on STM32L476.

# Problems, todos, notes
## General
* [x] The system tick is not working before the scheduler is created, so stm32-hal routines can block indefinitely. 
  * My solution to this is is to bypass every HAL function that uses a timeout. HAL uses its own "locking" mechanisms, so they also have to be removed anyway (I see no point of having RTOS semaphores and HAL __HAL_LOCK / __HAL_UNLOCK calls).

# UART
* [ ] Change implementation to task notifications.
* [ ] Use other uarts as well.
* [ ] Connect one to another and write some automatic tests.
* [ ] Resturn bool from send and receive. 
* [ ] Change Status enum to Error, and provide bitwise operations.
* [ ] Add UART status query function. Return Error from there.
* [ ] Receive line should also return some bool value (false in case of a timeout).
* [ ] Template argument for receiveLine. A container to return.
