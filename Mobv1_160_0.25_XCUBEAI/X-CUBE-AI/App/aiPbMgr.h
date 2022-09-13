/**
 ******************************************************************************
 * @file    aiPbMgr.h
 * @author  MCD/AIS Team
 * @brief   Helper function for AI ProtoBuffer support
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019,2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef _AI_PB_MGR_H_
#define _AI_PB_MGR_H_

#include "ai_platform.h"

#include "pb.h"
#include "stm32msg.pb.h"

#ifndef AI_PB_TEST
#define AI_PB_TEST 0
#endif

/* AI_PB_FULL_IO - force the upload of the ai_buffer with AI_BUFFER_FMT_FLAG_IS_IO flag
 *                 (see aiPbMgrSendAiBuffer3() function) */
#ifndef AI_PB_FULL_IO
#define AI_PB_FULL_IO 0
#endif

#ifdef __cplusplus
extern "C" {
#endif


/* --------------------------- */

#if TF_LITE_STATIC_MEMORY /* C-define used to build the TFLM 2.3 files */
#if !defined(TFLM_RUNTIME)
#define TFLM_RUNTIME 1
#endif
#endif

/* Special definition */
struct tflm_c_buffer_extra {
  float scale;
  int zero_point;
};

struct tflm_c_buffer {
  ai_buffer buffer;
  struct tflm_c_buffer_extra extra;
  ai_shape_dimension shape_data[4];
};

struct tflm_c_report {
  ai_network_report report;
  ai_shape_dimension shape_data_act[4];
  ai_shape_dimension shape_data_params[4];
};


/* --------------------------- */

typedef struct _aiPbCmdFunc {
        EnumCmd cmd;
        void (*process)(const reqMsg *req, respMsg *resp, void *param);
        void *param;
} aiPbCmdFunc;

void aiPbMgrInit(const aiPbCmdFunc *funcs);

int aiPbMgrWaitAndProcess(void);

/* --------------------------- */

void aiPbMgrSendAck(const reqMsg *req, respMsg *resp,
        EnumState state, uint32_t param, EnumError error);

void aiPbMgrSendResp(const reqMsg *req, respMsg *resp, EnumState state);

bool aiPbMgrReceiveAiBuffer3(const reqMsg *req, respMsg *resp,
        EnumState state, ai_buffer *buffer);

#define PB_BUFFER_TYPE_SEND_WITHOUT_DATA ((uint32_t)(1U << 31))

bool aiPbMgrSendAiBuffer4(const reqMsg *req, respMsg *resp, EnumState state,
    uint32_t type, uint32_t id, ai_float dur_ms, const ai_buffer *buffer,
    ai_float scale, ai_i32 zero_point);

bool aiPbMgrSendLog(const reqMsg *req, respMsg *resp,
        EnumState state, uint32_t lvl, const char *str);

bool aiPbMgrSendLogV2(EnumState state, uint32_t lvl, const char *str);

void aiPbMgrSendNNInfo(const reqMsg *req, respMsg *resp,
        EnumState state, const ai_network_report *nn);

bool aiPbMgrWaitAck(void);

/* --------------------------- */

uint32_t aiPbAiBufferSize(const ai_buffer *buffer);
void aiPbStrCopy(const char *src, char *dst, uint32_t max);
uint32_t aiPbVersionToUint32(const ai_platform_version *ver);

/* --------------------------- */

void aiPbCmdSync(const reqMsg *req, respMsg *resp, void *param);
#define AI_PB_CMD_SYNC(par) { EnumCmd_CMD_SYNC, &aiPbCmdSync, (par) }

void aiPbCmdSysInfo(const reqMsg *req, respMsg *resp, void *param);
#define AI_PB_CMD_SYS_INFO(par) { EnumCmd_CMD_SYS_INFO, &aiPbCmdSysInfo, (par) }

#define AI_PB_CMD_END      { (EnumCmd)0, NULL, NULL }

#if defined(AI_PB_TEST) && (AI_PB_TEST == 1)
void aiPbTestCmd(const reqMsg *req, respMsg *resp, void *param);
#define AI_PB_CMD_TEST(par) { EnumCmd_CMD_TEST, &aiPbTestCmd, (par) }
#endif


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _AI_PB_MGR_H_ */
