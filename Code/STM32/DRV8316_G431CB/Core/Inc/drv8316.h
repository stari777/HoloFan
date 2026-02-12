/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef _DRV8316_H_
#define _DRV8316_H_

//! \file   drivers/drvic/drv8316/src/32b/f28x/f2805x/drv8316.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8316 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include <math.h>

// drivers
#include "spi.h"
#include "gpio.h"
#include "stdbool.h"
#include "stdint.h"
//#include "sw/drivers/spi/src/32b/f28x/f2805x/spi.h"
//#include "sw/drivers/gpio/src/32b/f28x/f2805x/gpio.h"

// modules

// solutions


//!
//! \defgroup DRVIC

//!
//! \ingroup DRVIC
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// DRV8316R defines

#define DRV8316_ADDR_MASK               (0x7E00)
#define DRV8316_DATA_MASK               (0x00FF)
#define DRV8316_RW_MASK                 (0x8000)
#define DRV8316_FAULT_TYPE_MASK         (0x00FF)

#define DRV8316_IC_STAT00_FAULT_BITS      (1 << 0)
#define DRV8316_IC_STAT00_OT_BITS         (1 << 1)
#define DRV8316_IC_STAT00_OVP_BITS        (1 << 2)
#define DRV8316_IC_STAT00_NPOR_BITS       (1 << 3)
#define DRV8316_IC_STAT00_OCP_BITS        (1 << 4)
#define DRV8316_IC_STAT00_SPI_FLT_BITS    (1 << 5)
#define DRV8316_IC_STAT00_BK_FLT_BITS     (1 << 6)
#define DRV8316_IC_STAT00_RESERVED_BITS   (1 << 7)

#define DRV8316_STAT01_OCP_LA_BITS   (1 << 0)
#define DRV8316_STAT01_OCP_HA_BITS   (1 << 1)
#define DRV8316_STAT01_OCP_LB_BITS   (1 << 2)
#define DRV8316_STAT01_OCP_HB_BITS   (1 << 3)
#define DRV8316_STAT01_OCP_LC_BITS   (1 << 4)
#define DRV8316_STAT01_OCP_HC_BITS   (1 << 5)
#define DRV8316_STAT01_OTS_BITS      (1 << 6)
#define DRV8316_STAT01_OTW_BITS      (1 << 7)

#define DRV8316_STAT02_SPI_ADDR_FLT_BITS   (1 << 0)
#define DRV8316_STAT02_SPI_SCLK_FLT_BITS   (1 << 1)
#define DRV8316_STAT02_SPI_PARITY_BITS     (1 << 2)
#define DRV8316_STAT02_VCP_UV_BITS         (1 << 3)
#define DRV8316_STAT02_BUCK_UV_BITS        (1 << 4)
#define DRV8316_STAT02_BUCK_OCP_BITS       (1 << 5)
#define DRV8316_STAT02_OTP_ERR_BITS        (1 << 6)
#define DRV8316_STAT02_RESERVED_BITS       (1 << 7)

#define DRV8316_CTRL03_REG_LOCK_BITS       (7 << 0)
#define DRV8316_CTRL03_RESERVED1_BITS      (1 << 3)
#define DRV8316_CTRL03_RESERVED2_BITS      (1 << 4)
#define DRV8316_CTRL03_RESERVED3_BITS      (1 << 5)
#define DRV8316_CTRL03_RESERVED4_BITS      (1 << 6)
#define DRV8316_CTRL03_RESERVED5_BITS      (1 << 7)

#define DRV8316_CTRL04_CLR_FLT_BITS        (1 << 0)
#define DRV8316_CTRL04_PWM_MODE_BITS       (3 << 1)
#define DRV8316_CTRL04_SLEW_BITS           (3 << 3)
#define DRV8316_CTRL04_RESERVED1_BITS      (1 << 5)
#define DRV8316_CTRL04_RESERVED2_BITS      (1 << 6)
#define DRV8316_CTRL04_RESERVED3_BITS      (1 << 7)

#define DRV8316_CTRL05_OTW_REP_BITS       (1 << 0)
#define DRV8316_CTRL05_SPI_FLT_REP_BITS   (1 << 1)
#define DRV8316_CTRL05_OVP_EN_BITS        (1 << 2)
#define DRV8316_CTRL05_OVP_SEL_BITS       (1 << 3)
#define DRV8316_CTRL05_RESERVED1_BITS      (1 << 4)
#define DRV8316_CTRL05_RESERVED2_BITS      (1 << 5)
#define DRV8316_CTRL05_RESERVED3_BITS      (1 << 6)
#define DRV8316_CTRL05_RESERVED4_BITS      (1 << 7)

#define DRV8316_CTRL06_OCP_MODE_BITS       (3 << 0)
#define DRV8316_CTRL06_OCP_LVL_BITS        (1 << 2)
#define DRV8316_CTRL06_OCP_RETRY_BITS      (1 << 3)
#define DRV8316_CTRL06_OCP_DEG_BITS        (3 << 4)
#define DRV8316_CTRL06_OCP_CBC_BITS        (1 << 6)
#define DRV8316_CTRL06_RESERVED_BITS       (1 << 7)

#define DRV8316_CTRL07_CSA_GAIN_BITS       (3 << 0)
#define DRV8316_CTRL07_EN_ASR_BITS         (1 << 2)
#define DRV8316_CTRL07_EN_AAR_BITS         (1 << 3)
#define DRV8316_CTRL07_AD_COMP_TH_HS_BITS  (1 << 4)
#define DRV8316_CTRL07_AD_COMP_TH_LS_BITS  (1 << 5)
#define DRV8316_CTRL07_ILIM_RECIR_BITS     (1 << 6)
#define DRV8316_CTRL07_BEMF_TH_BITS        (1 << 7)

#define DRV8316_CTRL08_BUCK_DIS_BITS       (1 << 0)
#define DRV8316_CTRL08_BUCK_SEL_BITS       (3 << 1)
#define DRV8316_CTRL08_BUCK_CL_BITS        (1 << 3)
#define DRV8316_CTRL08_BUCK_PS_DIS_BITS    (1 << 4)
#define DRV8316_CTRL08_BUCK_SR_BITS        (1 << 5)
#define DRV8316_CTRL08_RESERVED1_BITS      (1 << 6)
#define DRV8316_CTRL08_RESERVED2_BITS      (1 << 7)


// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum
{
    DRV8316_CTRLMODE_READ = 1 << 15,   //!< Read Mode
    DRV8316_CTRLMODE_WRITE = 0 << 15   //!< Write Mode
} DRV8316_CtrlMode_e;


//IC_STAT enumeration

typedef enum
{
  FAULT        = (1 << 0),
  OT           = (1 << 1),
  OVP          = (1 << 2),
  NPOR         = (1 << 3),
  OCP          = (1 << 4),
  SPI_FLT      = (1 << 5),
  BK_FLT       = (1 << 6),
  IC_STAT_RSV  = (1 << 7)
} DRV8316_IC_STAT_e;

//STAT1 enumeration

typedef enum
{
  OCP_LA   = (1 << 0),
  OCP_HA   = (1 << 1),
  OCP_LB   = (1 << 2),
  OCP_HB   = (1 << 3),
  OCP_LC   = (1 << 4),
  OCP_HC   = (1 << 5),
  OTS      = (1 << 6),
  OTW      = (1 << 7),
} DRV8316_STAT1_e;

//STAT2 enumeration

typedef enum
{
    SPI_ADDR_FLT   = (1 << 0),
    SPI_SCLK_FLT   = (1 << 1),
    SPI_PARITY     = (1 << 2),
    VCP_UV         = (1 << 3),
    BUCK_UV        = (1 << 4),
    BUCK_OCP       = (1 << 5),
    OTP_ERR        = (1 << 6),
    STAT02_RSV     = (1 << 7),
} DRV8316_STAT2_e;

//CTRL03 RegLock enumeration

typedef enum
{
    Reg_Lock_UnlockAll    = (3 << 0),
    Reg_Lock_LockAll      = (6 << 0)

} DRV8316_CTRL03_RegLock_e;

//CTRL04 PWM_Mode enumeration                        //enum for whole register or just field? CLR_FLT would be left alone

typedef enum
{
    PWM_6x_Mode          = (0 << 1),
    PWM_6x_Mode_Limit    = (1 << 1),
    PWM_3x_Mode          = (2 << 1),
    PWM_3x_Mode_Limit    = (3 << 1),
} DRV8316_CTRL04_PwmMode_e;

//CTRL04 SlewRate enumeration

typedef enum
{
    SlewRate_25          = (0 << 3),
    SlewRate_50          = (1 << 3),
    SlewRate_150         = (2 << 3),
    SlewRate_200         = (3 << 3)
} DRV8316_CTRL04_SlewRate_e;


//CTRL05 Overtemp enumeration

typedef enum
{
    Overtemp_Dis           = (0 << 0),
    Overtemp_En            = (1 << 0),
} DRV8316_CTRL05_Overtemp_e;


//CTRL05 SPI_FLT_REP enumeration

typedef enum
{
    SPI_Fault_Reporting_En          = (0 << 1),
    SPI_Fault_Reporting_Dis         = (1 << 1),
} DRV8316_CTRL05_SPI_FLT_REP_e;

//CTRL05 Overvoltage_E enumeration

typedef enum
{
    Overvoltage_Dis        = (0 << 2),
    Overvoltage_En         = (1 << 2),
} DRV8316_CTRL05_Ovp_e;

//CTRL05 OVervoltageLevel enumeration

typedef enum
{
    Overvoltage_Level_32V  = (0 << 3),
    Overvoltage_Level_20V  = (1 << 3)
} DRV8316_CTRL05_OvpLevel_e;


//CTRL06 OCP_Mode enumeration

typedef enum
{
    Latched_Fault    = (0 << 0),
    Automatic_Retry  = (1 << 0),
    Report_Only      = (2 << 0),
    No_Action        = (3 << 0),
} DRV8316_CTRL06_OcpMode_e;

//CTRL06 OCP_Level enumeration

typedef enum
{
    Overcurrent_Level_10A        = (0 << 2),
    Overcurrent_Level_15A        = (1 << 2),
} DRV8316_CTRL06_OcpLevel_e;

//CTRL06 OCP Retry Time enumeration

typedef enum
{
    Overcurrent_RetryTime_1ms    = (0 << 3),
    Overcurrent_RetryTime_500ms  = (1 << 3),
} DRV8316_CTRL06_RetryTime_e;

//CTRL06 OCP Deglitch Time enumeration

typedef enum
{
    Overcurrent_Deglitch_0p2us   = (0 << 4),
    Overcurrent_Deglitch_1us     = (1 << 4),
    Overcurrent_Deglitch_4us     = (2 << 4),
    Overcurrent_Deglitch_8us     = (3 << 4),
} DRV8316_CTRL06_OcpDeglitch_e;

//CTRL06 OCP PWM cycle operation enumeration

typedef enum
{
    Overcurrent_CBC_Dis          = (0 << 6),
    Overcurrent_CBC_En           = (1 << 6)
} DRV8316_CTRL06_PwmCycleOperation_e;



//CTRL07 CSA gain enumeration

typedef enum
{
    CSA_Gain_0p15                 = (0 << 0),
    CSA_Gain_0p1875               = (1 << 0),
    CSA_Gain_0p25                 = (2 << 0),
    CSA_Gain_0p375                = (3 << 0),
} DRV8316_CTRL07_CsaGain_e;

//CTRL07 Active synchronous rectification enumeration

typedef enum
{
    ASR_Dis                       = (0 << 2),
    ASR_En                        = (1 << 2),
} DRV8316_CTRL07_ASR_e;

//CTRL07 Active asynchronous rectification enumeration

typedef enum
{
    AAR_Dis                       = (0 << 3),
    AAR_En                        = (1 << 3),
} DRV8316_CTRL07_AAR_e;

//CTRL07 Active demagnitization threshold HS enumeration

typedef enum
{
    Active_Demag_Thres_HS_100mA   = (0 << 4),
    Active_Demag_Thres_HS_150mA   = (1 << 4),
} DRV8316_CTRL07_ActiveDemagTh_HS_e;

//CTRL07 Active demagnitization threshold LS enumeration

typedef enum
{
    Active_Demag_Thres_LS_100mA   = (0 << 5),
    Active_Demag_Thres_LS_150mA   = (1 << 5),
} DRV8316_CTRL07_ActiveDemagTh_LS_e;

//CTRL07 Current limit recirculation enumeration

typedef enum
{
    Brake_Mode                    = (0 << 6),
    Coast_Mode                    = (1 << 6),
} DRV8316_CTRL07_IlimRecir_e;

//CTRL07 Back-EMF threshold enumeration

typedef enum
{
    BEMF_Thres_1                  = (0 << 7),
    BEMF_Thres_2                  = (1 << 7)
} DRV8316_CTRL07_BemfTh_e;

//CTRL08 buck disable enumeration

typedef enum
{
    Buck_En                  = (0 << 0),
    Buck_Dis                 = (1 << 0),
} DRV8316_CTRL08_BuckDis_e;

//CTRL08 buck select voltage enumeration

typedef enum
{
    Buck_Voltage_3p3V        = (0 << 1),
    Buck_Voltage_5p0V        = (1 << 1),
    Buck_Voltage_4p0V        = (2 << 1),
    Buck_Voltage_5p7V        = (3 << 1),
} DRV8316_CTRL08_BuckSel_e;

//CTRL08 buck current limit enumeration

typedef enum
{
    Buck_Current_Limit_600mA = (0 << 3),
    Buck_Current_Limit_150mA = (1 << 3),
} DRV8316_CTRL08_BuckCurLim_e;

//CTRL08 buck power sequence disable enumeration

typedef enum
{
    Buck_Power_Seq_En        = (0 << 4),
    Buck_Power_Seq_Dis       = (1 << 4),
} DRV8316_CTRL08_BuckPwrSeqDis_e;

//CTRL08 buck slew rate enumeration

typedef enum
{
    Buck_SlewRate_1000V_us   = (0 << 5),
    Buck_SlewRate_200V_us    = (1 << 5)
} DRV8316_CTRL08_BuckSlewRate_e;

//! \brief Enumeration for the register addresses   //what should the bit field position be?
//!
typedef enum
{
    DRV8316_ADDRESS_IC_STAT_0  = (0 << 9),  //!< IC_Stat
    DRV8316_ADDRESS_STATUS_1  = (1 << 9),  //!< Status Register 1
    DRV8316_ADDRESS_STATUS_2  = (2 << 9),  //!< Status Register 2
    DRV8316_ADDRESS_CONTROL_1 = (3 << 9),  //!< Control Register 1
    DRV8316_ADDRESS_CONTROL_2 = (4 << 9),  //!< Control Register 2
    DRV8316_ADDRESS_CONTROL_3 = (5 << 9),  //!< Control Register 3
    DRV8316_ADDRESS_CONTROL_4 = (6 << 9),   //!< Control Register 4
    DRV8316_ADDRESS_CONTROL_5 = (7 << 9),   //!< Control Register 5
    DRV8316_ADDRESS_CONTROL_6 = (8 << 9)   //!< Control Register 6
} DRV8316_Address_e;


//-----------------------OBJECTS-----------------------------


//! \brief Object for the DRV8316 IC_STAT register
//!
typedef struct _DRV8316_IC_Stat00_t
{
  bool                  FAULT;        // Bits 0
  bool                  OT;           // Bits 1
  bool                  OVP;          // Bits 2
  bool                  NPOR;         // Bits 3
  bool                  OCP;          // Bits 4
  bool                  SPI_FLT;      // Bits 5
  bool                  BK_FLT;       // Bits 6
  bool                  IC_STAT_RSV1;  // Bits 7
}DRV8316_IC_Stat00_t;

//! \brief Object for the DRV8316 STAT1 register
//!
typedef struct _DRV8316_Stat01_t_
{
  bool                  OCP_LA;       // Bits 0
  bool                  OCP_HA;       // Bits 1
  bool                  OCP_LB;       // Bits 2
  bool                  OCP_HB;       // Bits 3
  bool                  OCP_LC;       // Bits 4
  bool                  OCP_HC;       // Bits 5
  bool                  OTS;          // Bits 6
  bool                  OTW;          // Bits 7
}DRV8316_Stat01_t;


//! \brief Object for the DRV8316 STAT2 register
//!
typedef struct _DRV8316_Stat02_t_
{
  bool                  SPI_ADDR_FLT;      // Bits 0
  bool                  SPI_SCLK_FLT;      // Bits 1
  bool                  SPI_PARITY;        // Bits 2
  bool                  VCP_UV;            // Bits 3
  bool                  BUCK_UV;           // Bits 4
  bool                  BUCK_OCP;          // Bits 5
  bool                  OTP_ERR;           // Bits 6
  bool                  STAT02_RSV1;       // Bits 7
}DRV8316_Stat02_t;


//! \brief Object for the DRV8316 CTRL1 register
//!
typedef struct _DRV8316_Ctrl03_t_
{
  DRV8316_CTRL03_RegLock_e  REG_LOCK;  // Bits 0-2
  bool                   CTRL03_RSV1;  // Bits 3
  bool                   CTRL03_RSV2;  // Bits 4
  bool                   CTRL03_RSV3;  // Bits 5
  bool                   CTRL03_RSV4;  // Bits 6
  bool                   CTRL03_RSV5;  // Bits 7
}DRV8316_Ctrl03_t;


//! \brief Object for the DRV8316 CTRL2 register
//!
typedef struct _DRV8316_Ctrl04_t_
{
  bool                          CLR_FLT;  // Bits 0
  DRV8316_CTRL04_PwmMode_e     PWM_MODE;  // Bits 1-2
  DRV8316_CTRL04_SlewRate_e        SLEW;  // Bits 3-4
  bool                      CTRL04_RSV1;  // Bits 5
  bool                      CTRL04_RSV2;  // Bits 6
  bool                      CTRL04_RSV3;  // Bits 7
}DRV8316_Ctrl04_t;


//! \brief Object for the DRV8316 CTRL3 register
//!
typedef struct _DRV8316_Ctrl05_t_
{
  bool                      OTW_REP;    // Bits 0
  bool                  SPI_FLT_REP;    // Bits 1
  bool                       OVP_EN;    // Bits 2
  bool                      OVP_SEL;    // Bits 3
  bool                  CTRL05_RSV1;    // Bits 4
  bool                  CTRL05_RSV2;    // Bits 5
  bool                  CTRL05_RSV3;    // Bits 6
  bool                  CTRL05_RSV4;    // Bits 7
}DRV8316_Ctrl05_t;


//! \brief Object for the DRV8316 CTRL4 register
//!
typedef struct _DRV8316_Ctrl06_t_
{
  DRV8316_CTRL06_OcpMode_e         OCP_MODE;    // Bits 0-1
  bool                              OCP_LVL;    // Bits 2
  bool                            OCP_RETRY;    // Bits 3
  DRV8316_CTRL06_OcpDeglitch_e      OCP_DEG;    // Bits 4-5
  bool                              OCP_CBC;    // Bits 6
  bool                           CTRL06_RSV;    // Bits 7
}DRV8316_Ctrl06_t;


//! \brief Object for the DRV8316 CTRL5 register
//!
typedef struct _DRV8316_Ctrl07_t_
{
  DRV8316_CTRL07_CsaGain_e        CSA_GAIN;    // Bits 0-1
  bool                              EN_ASR;    // Bits 2
  bool                              EN_AAR;    // Bits 3
  bool                       AD_COMP_TH_HS;    // Bits 4
  bool                       AD_COMP_TH_LS;    // Bits 5
  bool                          ILIM_RECIR;    // Bits 6
  bool                             BEMF_TH;    // Bits 7
}DRV8316_Ctrl07_t;


//! \brief Object for the DRV8316 CTRL6 register
//!
typedef struct _DRV8316_Ctrl08_t_
{
  bool                      BUCK_DIS;       // Bits 0
  DRV8316_CTRL08_BuckSel_e  BUCK_SEL;       // Bits 1-2
  bool                      BUCK_CL;        // Bits 3
  bool                      BUCK_PS_DIS;    // Bits 4
  bool                      BUCK_SR;        // Bits 5
  bool                      CTRL08_RSV1;    // Bits 6
  bool                      CTRL08_RSV2;    // Bits 7
}DRV8316_Ctrl08_t;


//! \brief Object for the DRV8316 registers and commands
//!
typedef struct _DRV8316_SPIVars_t_
{
    DRV8316_IC_Stat00_t      IC_Stat_Reg_00;
    DRV8316_Stat01_t         Stat_Reg_01;
    DRV8316_Stat02_t         Stat_Reg_02;
    DRV8316_Ctrl03_t          Ctrl_Reg_03;
    DRV8316_Ctrl04_t          Ctrl_Reg_04;
    DRV8316_Ctrl05_t          Ctrl_Reg_05;
    DRV8316_Ctrl06_t          Ctrl_Reg_06;
    DRV8316_Ctrl07_t          Ctrl_Reg_07;
    DRV8316_Ctrl08_t          Ctrl_Reg_08;
    bool                      writeCmd;
    bool                      readCmd;

    uint16_t                  manWriteAddr;
    uint16_t                  manReadAddr;
    uint16_t                  manWriteData;
    uint16_t                  manReadData;
    bool                      manWriteCmd;
    bool                      manReadCmd;
}DRV8316_SPIVars_t;


//! \brief Defines the DRV8316 object
//!
typedef struct _DRV8316_Obj_
{
    uint32_t  spiHandle;     //!< handle for the serial peripheral interface
    uint32_t  gpioNumber_CS; //!< GPIO connected to the DRV8316 CS pin
    uint32_t  gpioNumber_EN; //!< GPIO connected to the DRV8316 enable pin
    bool      rxTimeOut;     //!< timeout flag for the RX FIFO
    bool      enableTimeOut; //!< timeout flag for DRV8316 enable
} DRV8316_Obj;


//! \brief Defines the DRV8316 handle
//!
typedef struct _DRV8316_Obj_ *DRV8316_Handle;


//! \brief Defines the DRV8316 Word type
//!
typedef  uint16_t    DRV8316_Word_t;


// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the DRV8316 object
//! \param[in] pMemory   A pointer to the memory for the DRV8316 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8316
//!                      object, bytes
//! \return    The DRV8316 object handle
extern DRV8316_Handle DRV8316_init(void *pMemory);

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8316_Word_t DRV8316_buildCtrlWord(
                                            const DRV8316_CtrlMode_e ctrlMode,
                                            const DRV8316_Address_e regAddr,
                                            const uint16_t data)
{
    uint16_t p_addr = regAddr;
    uint16_t p_data = data;
    uint16_t p_mode = ctrlMode;

    uint16_t calc = (p_mode & 0x8000) | (p_addr & 0x7E00) | (p_data & 0x00FF);
    uint16_t parity = 0;
    while(calc)
    {
        parity ^= (calc & 1);
        calc >>= 1;
    }

    parity <<= 8;

    DRV8316_Word_t ctrlWord = ctrlMode | regAddr | parity | (data & DRV8316_DATA_MASK);

    return(ctrlWord);
} // end of DRV8316_buildCtrlWord() function

//! \brief     Enables the DRV8316
//! \param[in] handle     The DRV8316 handle
extern void DRV8316_enable(DRV8316_Handle handle);

//! \brief     Sets the SPI handle in the DRV8316
//! \param[in] handle     The DRV8316 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8316_setSPIHandle(DRV8316_Handle handle,uint32_t spiHandle);

//! \brief     Sets the GPIO number in the DRV8316
//! \param[in] handle       The DRV8316 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8316_setGPIOCSNumber(DRV8316_Handle handle,uint32_t gpioNumber);

//! \brief     Sets the GPIO number in the DRV8316
//! \param[in] handle       The DRV8316 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8316_setGPIONumber(DRV8316_Handle handle,uint32_t gpioNumber);

//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8316 handle
static inline void DRV8316_resetEnableTimeout(DRV8316_Handle handle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    obj->enableTimeOut = false;

    return;
} // end of DRV8316_resetEnableTimeout() function

//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8316 handle
static inline void DRV8316_resetRxTimeout(DRV8316_Handle handle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    obj->rxTimeOut = false;

    return;
} // end of DRV8316_resetRxTimeout() function

//! \brief     Initialize the interface to all 8316 SPI variables
//! \param[in] handle  The DRV8316 handle
extern void DRV8316_setupSPI(DRV8316_Handle handle,
                             DRV8316_SPIVars_t *drv8316SPIVars);

//! \brief     Reads data from the DRV8316 register
//! \param[in] handle   The DRV8316 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint16_t DRV8316_readSPI(DRV8316_Handle handle,
                                const DRV8316_Address_e regAddr);

//! \brief     Writes data to the DRV8316 register
//! \param[in] handle   The DRV8316 handle
//! \param[in] regAddr  The register name
//! \param[in] data     The data value
extern void DRV8316_writeSPI(DRV8316_Handle handle,
                             const DRV8316_Address_e regAddr,
                             const uint16_t data);

//! \brief     Write to the DRV8316 SPI registers
//! \param[in] handle  The DRV8316 handle
//! \param[in] drv8316SPIVars  The (DRV8316_SPIVars_t) structure that contains
//!                           all DRV8316 Status/Control register options
extern void DRV8316_writeData(DRV8316_Handle handle,
                              DRV8316_SPIVars_t *drv8316SPIVars);

//! \brief     Read from the DRV8316 SPI registers
//! \param[in] handle  The DRV8316 handle
//! \param[in] drv8316SPIVars  The (DRV8316_SPIVars_t) structure that contains
//!                           all DRV8316 Status/Control register options
extern void DRV8316_readData(DRV8316_Handle handle,
                             DRV8316_SPIVars_t *drv8316SPIVars);

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of DRV8316_H definition
