/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _TUSB_OSAL_CHIBIOS_H_
#define _TUSB_OSAL_CHIBIOS_H_

// ChibiOS Headers
#include "ch.h"

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// TASK API
//--------------------------------------------------------------------+
static inline void osal_task_delay(uint32_t msec)
{
  chThdSleepMilliseconds(msec);
}

//--------------------------------------------------------------------+
// Semaphore API
//--------------------------------------------------------------------+
typedef struct {
  uint16_t    size;
  semaphore_t sem;
} osal_semaphore_def_t;
typedef osal_semaphore_def_t * osal_semaphore_t;

static inline osal_semaphore_t osal_semaphore_create(osal_semaphore_def_t* semdef)
{
  chSemObjectInit(&semdef->sem, semdef->size);
  return semdef;
}

static inline bool osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr)
{
  if ( !in_isr )
  {
    chSemSignal(&sem_hdl->sem);
    return true;
  }
  else
  {
    chSysLockFromISR();
    chSemSignalI(&sem_hdl->sem);
    chSysUnlockFromISR();
    return true;
  }
}

static inline bool osal_semaphore_wait (osal_semaphore_t sem_hdl, uint32_t msec)
{
  const sysinterval_t ticks = (msec == OSAL_TIMEOUT_WAIT_FOREVER) ? TIME_INFINITE : TIME_MS2I(msec);
  return chSemWaitTimeout(&sem_hdl->sem, ticks) == MSG_OK;
}

static inline void osal_semaphore_reset(osal_semaphore_t const sem_hdl)
{
  chSemReset(&sem_hdl->sem, sem_hdl->size);
}

//--------------------------------------------------------------------+
// MUTEX API (priority inheritance)
//--------------------------------------------------------------------+
typedef osal_semaphore_def_t osal_mutex_def_t;
typedef osal_semaphore_t osal_mutex_t;

static inline osal_mutex_t osal_mutex_create(osal_mutex_def_t* mdef)
{
  chSemObjectInit(&mdef->sem, mdef->size);
  return mdef;
}

static inline bool osal_mutex_lock (osal_mutex_t mutex_hdl, uint32_t msec)
{
  uint32_t const ticks = (msec == OSAL_TIMEOUT_WAIT_FOREVER) ? TIME_INFINITE : TIME_MS2I(msec);
  return chSemWaitTimeout(&mutex_hdl->sem, ticks) == MSG_OK;
}

static inline bool osal_mutex_unlock(osal_mutex_t mutex_hdl)
{
  chSemSignal(&mutex_hdl->sem);
  return true;
}

//--------------------------------------------------------------------+
// QUEUE API
//--------------------------------------------------------------------+

// role device/host is used by OS NONE for mutex (disable usb isr) only
#define OSAL_QUEUE_DEF(_role, _name, _depth, _type) \
  static _type _name##_##objbuf[_depth];\
  static msg_t _name##_##msgbuf[_depth];\
  osal_queue_def_t _name = { .depth = _depth, .obj_sz = sizeof(_type), .objbuf = _name##_##objbuf, .msgbuf = _name##_##msgbuf };

// Use FIFO as queue (mailbox + memory pool)
typedef struct
{
  uint16_t  depth;
  uint16_t  obj_sz;
  void*     objbuf;
  msg_t*    msgbuf;

  objects_fifo_t fifo;
} osal_queue_def_t;
typedef osal_queue_def_t * osal_queue_t;

static inline osal_queue_t osal_queue_create(osal_queue_def_t* qdef)
{
  chFifoObjectInit(&qdef->fifo, qdef->obj_sz, qdef->depth, qdef->objbuf, qdef->msgbuf);
  return qdef;
}

static inline bool osal_queue_receive(osal_queue_t qhdl, void* data)
{
  void* objpp; // pointer to the object

  bool r = chFifoReceiveObjectTimeout(&qhdl->fifo, &objpp, TIME_MS2I(10000)) != MSG_OK;
  if (r)
    return false;

  memcpy(data, objpp, qhdl->obj_sz);
  chFifoReturnObject(&qhdl->fifo, objpp);
  return true;
}

static inline bool osal_queue_send(osal_queue_t qhdl, void const * data, bool in_isr)
{
  void* obj;

  if ( !in_isr )
  {
    obj = chFifoTakeObjectTimeout(&qhdl->fifo, TIME_MS2I(10000));
    if (obj == NULL) // Allocation failed
      return false;
    memcpy(obj, data, qhdl->obj_sz);
    chFifoSendObject(&qhdl->fifo, obj);
    return true;
  }
  else
  {
    chSysLockFromISR();
    obj = chFifoTakeObjectI(&qhdl->fifo);
    chSysUnlockFromISR();

    if (obj == NULL) // Allocation failed
      return false;
    memcpy(obj, data, qhdl->obj_sz);

    chSysLockFromISR();
    chFifoSendObjectI(&qhdl->fifo, obj);
    chSysUnlockFromISR();
    return true;
  }
}

static inline bool osal_queue_empty(osal_queue_t qhdl)
{
  uint16_t cnt;
  chSysLock();
  cnt = chMBGetFreeCountI(&qhdl->fifo.mbx);
  chSysUnlock();
  return cnt == 0;
}

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_OSAL_FREERTOS_H_ */
