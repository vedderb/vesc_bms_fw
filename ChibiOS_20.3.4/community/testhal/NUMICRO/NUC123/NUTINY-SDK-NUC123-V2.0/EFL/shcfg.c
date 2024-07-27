/*
    Copyright (C) 2021 Alex Lewontin

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    shcfg.c
 * @brief   Shell config.
 *
 * @addtogroup Shell
 * @{
 */
#include "hal.h"
#include "shcfg.h"
#include "chprintf.h"

#include <string.h>
#include <stdlib.h>

MFSDriver mfsd;

const MFSConfig mfsd_config = {.flashp        = (BaseFlash *)&EFLD1,
                               .erased        = 0xFFFFFFFF,
                               .bank0_sectors = 4,
                               .bank0_start   = 0,
                               .bank1_sectors = 4,
                               .bank1_start   = 4,
                               .bank_size     = 2048};

void sh_kvs_put(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc < 2) {
        chprintf(chp, "Format: kvs_put key [value]\nAt this time, key must be numeric.\n");
        return;
    }
    mfs_id_t rid = atoi(argv[0]);
    if (rid < 1 || MFS_CFG_MAX_RECORDS < rid) {
      chprintf(chp, "key must be [%d, %d].\n", 1, MFS_CFG_MAX_RECORDS);
      return;
    }
    mfsWriteRecord(&mfsd, rid, strlen(argv[1]), (uint8_t *)argv[1]);
}

void sh_kvs_get(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc < 1) {
    chprintf(chp,
             "Format: kvs_get key\nAt this time, key must be "
             "numeric.\n");
    return;
  }
  mfs_id_t rid = atoi(argv[0]);
  if (rid < 1 || MFS_CFG_MAX_RECORDS < rid) {
    chprintf(chp, "key must be [%d, %d].\n", 1, MFS_CFG_MAX_RECORDS);
    return;
  }

  uint8_t buf[128];
  size_t  n = 128;
  mfs_error_t err = mfsReadRecord(&mfsd, rid, &n, buf);
  switch (err) {
    case MFS_WARN_GC:
    case MFS_WARN_REPAIR:
    case MFS_NO_ERROR:
      chprintf(chp, "%.*s\n", n, buf);
      break;
    case MFS_ERR_NOT_FOUND:
      chprintf(chp, "Record not found\n");
      break;
    default:
      chprintf(chp, "Unknown error reading record: %d\n", err);
  }
}

const char all_flag[] = "--all";
void sh_kvs_erase(BaseSequentialStream *chp, int argc, char *argv[]) {

  if (argc < 1) {
    chprintf(chp,
             "Format: kvs_erase [%s] key\nAt this time, key must be"
             "numeric.\n", all_flag);
    return;
  }

  if (strcmp(all_flag, argv[0]) == 0) {
    mfsErase(&mfsd);
  } else {
    mfs_id_t rid = atoi(argv[0]);
    if (rid < 1 || MFS_CFG_MAX_RECORDS < rid) {
      chprintf(chp, "key must be [%d, %d].\n", 1, MFS_CFG_MAX_RECORDS);
      return;
    }
    mfsEraseRecord(&mfsd, rid);
  }

}

const ShellCommand commands[] = {{"kvs_put", sh_kvs_put},
                                   {"kvs_get", sh_kvs_get},
                                   {"kvs_erase", sh_kvs_erase},
                                   {NULL, NULL}};

const ShellConfig shell_cfg = {(BaseSequentialStream *)&SHELL_SERIAL_DRIVER, commands};
