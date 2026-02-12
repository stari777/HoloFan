//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:59 CDT 2020 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//! \file   solutions/boostxl_drv8316rs/f28004x/drivers/source/drv8316.c
//! \brief  Contains the various functions related to the DRV8316 object
//!

// **************************************************************************
// the includes

#include <math.h>

// **************************************************************************
// drivers
#include "drv8316.h"

// **************************************************************************
// modules

// **************************************************************************
// platforms

// **************************************************************************
// the defines

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes
void DRV8316_enable(DRV8316_Handle handle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;
    volatile uint16_t enableWaitTimeOut;
    uint16_t n = 0;

    // Enable the DRV8316
    GPIO_writePin(obj->gpioNumber_EN, 0);

    enableWaitTimeOut = 0;

    // Make sure the FAULT bit is not set during startup
    while(((DRV8316_readSPI(handle, DRV8316_ADDRESS_IC_STAT_0) &
            DRV8316_IC_STAT00_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
    {
        if(++enableWaitTimeOut > 999)
        {
            obj->enableTimeOut = true;
        }
    }

    // Wait for the DRV8316 to go through start up sequence
    for(n = 0; n < 0xffff; n++)
    {
        __asm(" NOP");
    }

    return;
} // end of DRV8316_enable() function

DRV8316_Handle DRV8316_init(void *pMemory)
{
    DRV8316_Handle handle;

    // assign the handle
    handle = (DRV8316_Handle)pMemory;

    DRV8316_resetRxTimeout(handle);
    DRV8316_resetEnableTimeout(handle);

    return(handle);
} // end of DRV8316_init() function


//DRV8316_CTRL03_PeakSourCurHS_e DRV8316_getPeakSourCurHS(DRV8316_Handle handle)
//{
//    uint16_t data;
//
//    // read data
//    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_3);
//
//    // mask the bits
//    data &= DRV8316_CTRL03_IDRIVEP_HS_BITS;
//
//    return((DRV8316_CTRL03_PeakSourCurHS_e)data);
//} // end of DRV8316_getPeakSourCurHS function
//
//DRV8316_CTRL03_PeakSinkCurHS_e DRV8316_getPeakSinkCurHS(DRV8316_Handle handle)
//{
//    uint16_t data;
//
//    // read data
//    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_3);
//
//    // mask the bits
//    data &= DRV8316_CTRL03_IDRIVEN_HS_BITS;
//
//    return((DRV8316_CTRL03_PeakSinkCurHS_e)data);
//} // end of DRV8316_getPeakSinkCurHS function
//
//DRV8316_CTRL04_PeakTime_e DRV8316_getPeakSourTime(DRV8316_Handle handle)
//{
//    uint16_t data;
//
//    // read data
//    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_4);
//
//    // mask the bits
//    data &= DRV8316_CTRL04_TDRIVE_BITS;
//
//    return((DRV8316_CTRL04_PeakTime_e)data);
//} // end of DRV8316_getPeakSourTime function
//
//DRV8316_CTRL04_PeakSourCurLS_e DRV8316_getPeakSourCurLS(DRV8316_Handle handle)
//{
//    uint16_t data;
//
//    // read data
//    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_4);
//
//    // mask the bits
//    data &= DRV8316_CTRL04_IDRIVEP_LS_BITS;
//
//    return((DRV8316_CTRL04_PeakSourCurLS_e)data);
//} // end of DRV8316_getPeakSourCurLS function
//
//DRV8316_CTRL04_PeakSinkCurLS_e DRV8316_getPeakSinkCurLS(DRV8316_Handle handle)
//{
//    uint16_t data;
//
//    // read data
//    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_4);
//
//    // mask the bits
//    data &= DRV8316_CTRL04_IDRIVEN_LS_BITS;
//
//    return((DRV8316_CTRL04_PeakSinkCurLS_e)data);
//} // end of DRV8316_getPeakSinkCurLS function
//
//
//DRV8316_CTRL04_PeakSinkTime_e DRV8316_getPeakSinkTime(DRV8316_Handle handle)
//{
//  uint16_t data;
//
//  // read data
//  data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_4);
//
//  // mask the bits
//  data &= DRV8316_CTRL04_TDRIVE_BITS;
//
//  return((DRV8316_CTRL04_PeakSinkTime_e)data);
//} // end of DRV8316_getPeakSinkTime function
//
//
//DRV8316_CTRL05_OcpDeg_e DRV8316_getVDSDeglitch(DRV8316_Handle handle)
//{
//    uint16_t data;
//
//    // read data
//    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_5);
//
//    // mask the bits
//    data &= DRV8316_CTRL05_OCP_DEG_BITS;
//
//    return((DRV8316_CTRL05_OcpDeg_e)data);
//} // end of DRV8316_getVDSDeglitch function
//
//
//DRV8316_CTRL07_VDSBlanking_e DRV8316_getVDSBlanking(DRV8316_Handle handle)
//{
//  uint16_t data;
//
//  // read data
//  data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_7);
//
//  // mask the bits
//  data &= DRV8316_CTRL07_TBLANK_BITS;
//
//  return((DRV8316_CTRL07_VDSBlanking_e)data);
//} // end of DRV8316_getVDSBlanking function
//
//
//DRV8316_CTRL05_DeadTime_e DRV8316_getDeadTime(DRV8316_Handle handle)
//{
//    uint16_t data;
//
//    // read data
//    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_5);
//
//    // mask the bits
//    data &= DRV8316_CTRL05_DEAD_TIME_BITS;
//
//    return((DRV8316_CTRL05_DeadTime_e)data);
//} // end of DRV8316_getDeadTime function

DRV8316_CTRL04_PwmMode_e DRV8316_getPWMMode(DRV8316_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8316_readSPI(handle, DRV8316_ADDRESS_CONTROL_4);

    // mask the bits
    data &= DRV8316_CTRL04_PWM_MODE_BITS;

    return((DRV8316_CTRL04_PwmMode_e)data);
} // end of DRV8316_getPWMMode function

void DRV8316_setSPIHandle(DRV8316_Handle handle, uint32_t spiHandle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    // initialize the serial peripheral interface object
    obj->spiHandle = spiHandle;

    return;
} // end of DRV8316_setSPIHandle() function

void DRV8316_setGPIOCSNumber(DRV8316_Handle handle, uint32_t gpioNumber)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_CS = gpioNumber;

    return;
} // end of DRV8316_setGPIOCSNumber() function

void DRV8316_setGPIONumber(DRV8316_Handle handle, uint32_t gpioNumber)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_EN = gpioNumber;

    return;
} // end of DRV8316_setGPIONumber() function

void DRV8316_setupSPI(DRV8316_Handle handle,
                      DRV8316_SPIVars_t *drv8316SPIVars)
{
    DRV8316_Address_e drvRegAddr;
    uint16_t drvDataNew;

    // Set Default Values
    // Manual Read/Write
    drv8316SPIVars->manReadAddr  = 0;
    drv8316SPIVars->manReadData  = 0;
    drv8316SPIVars->manReadCmd = false;
    drv8316SPIVars->manWriteAddr = 0;
    drv8316SPIVars->manWriteData = 0;
    drv8316SPIVars->manWriteCmd = false;

    // Read/Write
    drv8316SPIVars->readCmd  = false;
    drv8316SPIVars->writeCmd = false;

    // Read registers for default values
    // Read IC Status Register                              //when do i need the ?1:0
    drvRegAddr = DRV8316_ADDRESS_IC_STAT_0;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316SPIVars->IC_Stat_Reg_00.FAULT         = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_FAULT_BITS)?1:0;
    drv8316SPIVars->IC_Stat_Reg_00.OT            = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_OT_BITS)?1:0;
    drv8316SPIVars->IC_Stat_Reg_00.OVP           = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_OVP_BITS)?1:0;
    drv8316SPIVars->IC_Stat_Reg_00.NPOR          = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_NPOR_BITS)?1:0;
    drv8316SPIVars->IC_Stat_Reg_00.SPI_FLT       = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_SPI_FLT_BITS)?1:0;
    drv8316SPIVars->IC_Stat_Reg_00.BK_FLT        = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_BK_FLT_BITS)?1:0;
    drv8316SPIVars->IC_Stat_Reg_00.IC_STAT_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_RESERVED_BITS)?1:0;

    // Read Status Register 1
    drv8316SPIVars->Stat_Reg_01.OCP_LA  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_LA_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_01.OCP_HA  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_HA_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_01.OCP_LB  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_LB_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_01.OCP_HB  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_HB_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_01.OCP_LC  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_LC_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_01.OCP_HC  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_HC_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_01.OTS     = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OTS_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_01.OTW     = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OTW_BITS)?1:0;

    // Read Status Register 2
    drv8316SPIVars->Stat_Reg_02.SPI_ADDR_FLT  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_SPI_ADDR_FLT_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_02.SPI_SCLK_FLT  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_SPI_SCLK_FLT_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_02.SPI_PARITY    = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_SPI_PARITY_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_02.VCP_UV        = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_VCP_UV_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_02.BUCK_UV       = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_BUCK_UV_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_02.BUCK_OCP      = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_BUCK_OCP_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_02.OTP_ERR       = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_OTP_ERR_BITS)?1:0;
    drv8316SPIVars->Stat_Reg_02.STAT02_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_RESERVED_BITS)?1:0;

    // Read Control Register 1
    drv8316SPIVars->Ctrl_Reg_03.REG_LOCK    = (DRV8316_CTRL03_RegLock_e)(drvDataNew & (uint16_t)DRV8316_CTRL03_REG_LOCK_BITS);
    drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED1_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED2_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED3_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED4_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED5_BITS)?1:0;

    // Read Control Register 2
    drv8316SPIVars->Ctrl_Reg_04.CLR_FLT      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_CLR_FLT_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_04.PWM_MODE     = (DRV8316_CTRL04_PwmMode_e)(drvDataNew & (uint16_t)DRV8316_CTRL04_PWM_MODE_BITS);
    drv8316SPIVars->Ctrl_Reg_04.SLEW         = (DRV8316_CTRL04_SlewRate_e)(drvDataNew & (uint16_t)DRV8316_CTRL04_SLEW_BITS);
    drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_RESERVED1_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_RESERVED2_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_RESERVED3_BITS)?1:0;

    // Read Control Register 3
    drv8316SPIVars->Ctrl_Reg_05.OTW_REP      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_OTW_REP_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_05.SPI_FLT_REP  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_SPI_FLT_REP_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_05.OVP_EN       = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_OVP_EN_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_05.OVP_SEL      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_OVP_SEL_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED1_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED2_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED3_BITS)?1:0;
    drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED4_BITS)?1:0;

    // Read Control Register 4
   drv8316SPIVars->Ctrl_Reg_06.OCP_MODE     = (DRV8316_CTRL06_OcpMode_e)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_MODE_BITS);
   drv8316SPIVars->Ctrl_Reg_06.OCP_LVL      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_LVL_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_06.OCP_RETRY    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_RETRY_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_06.OCP_DEG      = (DRV8316_CTRL06_OcpDeglitch_e)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_DEG_BITS);
   drv8316SPIVars->Ctrl_Reg_06.OCP_CBC      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_CBC_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_06.CTRL06_RSV   = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_RESERVED_BITS)?1:0;

   // Read Control Register 5
  drv8316SPIVars->Ctrl_Reg_07.CSA_GAIN       = (DRV8316_CTRL07_CsaGain_e)(drvDataNew & (uint16_t)DRV8316_CTRL07_CSA_GAIN_BITS);
  drv8316SPIVars->Ctrl_Reg_07.EN_ASR         = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_EN_ASR_BITS)?1:0;
  drv8316SPIVars->Ctrl_Reg_07.EN_AAR         = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_EN_AAR_BITS)?1:0;
  drv8316SPIVars->Ctrl_Reg_07.AD_COMP_TH_HS  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_AD_COMP_TH_HS_BITS)?1:0;
  drv8316SPIVars->Ctrl_Reg_07.AD_COMP_TH_LS  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_AD_COMP_TH_LS_BITS)?1:0;
  drv8316SPIVars->Ctrl_Reg_07.ILIM_RECIR     = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_ILIM_RECIR_BITS)?1:0;
  drv8316SPIVars->Ctrl_Reg_07.BEMF_TH        = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_BEMF_TH_BITS)?1:0;

   // Read Control Register 6
   drv8316SPIVars->Ctrl_Reg_08.BUCK_DIS       = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_DIS_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_08.BUCK_SEL       = (DRV8316_CTRL08_BuckSel_e)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_SEL_BITS);
   drv8316SPIVars->Ctrl_Reg_08.BUCK_CL        = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_CL_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_08.BUCK_PS_DIS    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_PS_DIS_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_08.BUCK_SR        = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_SR_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_08.CTRL08_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_RESERVED1_BITS)?1:0;
   drv8316SPIVars->Ctrl_Reg_08.CTRL08_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_RESERVED2_BITS)?1:0;

    return;
} // end of DRV8316_setupSPI() function

uint16_t DRV8316_readSPI(DRV8316_Handle handle,
                         const DRV8316_Address_e regAddr)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;
    const uint16_t data = 0;
    volatile uint16_t readWord;
    volatile uint16_t WaitTimeOut = 0;

    volatile SPI_RxFIFOLevel RxFifoCnt = SPI_FIFO_RXEMPTY;

    // build the control word
    ctrlWord = (uint16_t)DRV8316_buildCtrlWord(DRV8316_CTRLMODE_READ, regAddr, data);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);

//  GPIO_writePin(obj->gpioNumber_CS, 0);

    // wait for registers to update
    for(n = 0; n < 0x06; n++)
    {
        __asm(" NOP");
    }

    // write the command
    SPI_writeDataBlockingNonFIFO(obj->spiHandle, ctrlWord);

    // wait for two words to populate the RX fifo, or a wait timeout will occur
    while(RxFifoCnt < SPI_FIFO_RX1)
    {
        RxFifoCnt = SPI_getRxFIFOStatus(obj->spiHandle);

        if(++WaitTimeOut > 0xfffe)
        {
            obj->rxTimeOut = true;
        }
    }

    WaitTimeOut = 0xffff;

//  GPIO_writePin(obj->gpioNumber_CS, 1);

    // Read the word
    readWord = SPI_readDataNonBlocking(obj->spiHandle);

    return(readWord & DRV8316_DATA_MASK);
} // end of DRV8316_readSPI() function


void DRV8316_writeSPI(DRV8316_Handle handle, const DRV8316_Address_e regAddr,
                      const uint16_t data)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;

    // build the control word
    ctrlWord = (uint16_t)DRV8316_buildCtrlWord(DRV8316_CTRLMODE_WRITE, regAddr, data);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);

    //  GPIO_writePin(obj->gpioNumber_CS, 0);

    // wait for GPIO
    for(n = 0; n < 0x06; n++)
    {
        __asm(" NOP");
    }

    // write the command
    SPI_writeDataBlockingNonFIFO(obj->spiHandle, ctrlWord);

    // wait for registers to update
    for(n = 0; n < 0x40; n++)
    {
        __asm(" NOP");
    }

    //  GPIO_writePin(obj->gpioNumber_CS, 1);

    return;
}  // end of DRV8316_writeSPI() function


void DRV8316_writeData(DRV8316_Handle handle, DRV8316_SPIVars_t *drv8316SPIVars)
{
    DRV8316_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8316SPIVars->writeCmd)
    {
        // Write Control Register 1
        drvRegAddr = DRV8316_ADDRESS_CONTROL_1;
        drvDataNew = (drv8316SPIVars->Ctrl_Reg_03.REG_LOCK        ) | \
                     (drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV1 << 3) | \
                     (drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV2 << 4) | \
                     (drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV3 << 5) | \
                     (drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV4 << 6) | \
                     (drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV5 << 7);
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 2
        drvRegAddr = DRV8316_ADDRESS_CONTROL_2;
        drvDataNew = (drv8316SPIVars->Ctrl_Reg_04.CLR_FLT     << 0) | \
                     (drv8316SPIVars->Ctrl_Reg_04.PWM_MODE        ) | \
                     (drv8316SPIVars->Ctrl_Reg_04.SLEW            ) | \
                     (drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV1 << 5) | \
                     (drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV2 << 6) | \
                     (drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV3 << 7);
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 3
        drvRegAddr = DRV8316_ADDRESS_CONTROL_3;
        drvDataNew = (drv8316SPIVars->Ctrl_Reg_05.OTW_REP       << 0) | \
                    (drv8316SPIVars->Ctrl_Reg_05.SPI_FLT_REP    << 1) | \
                    (drv8316SPIVars->Ctrl_Reg_05.OVP_EN         << 2) | \
                    (drv8316SPIVars->Ctrl_Reg_05.OVP_SEL        << 3) | \
                    (drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV1    << 4) | \
                    (drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV2    << 5) | \
                    (drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV3    << 6) | \
                    (drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV3    << 7);
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 4
        drvRegAddr = DRV8316_ADDRESS_CONTROL_4;
        drvDataNew = (drv8316SPIVars->Ctrl_Reg_06.OCP_MODE)         | \
                     (drv8316SPIVars->Ctrl_Reg_06.OCP_LVL    << 2)  | \
                     (drv8316SPIVars->Ctrl_Reg_06.OCP_RETRY  << 3)  | \
                     (drv8316SPIVars->Ctrl_Reg_06.OCP_DEG)          | \
                     (drv8316SPIVars->Ctrl_Reg_06.OCP_CBC    << 6)  | \
                     (drv8316SPIVars->Ctrl_Reg_06.CTRL06_RSV << 7);
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 5
           drvRegAddr = DRV8316_ADDRESS_CONTROL_5;
           drvDataNew = (drv8316SPIVars->Ctrl_Reg_07.CSA_GAIN)            | \
                        (drv8316SPIVars->Ctrl_Reg_07.EN_ASR        << 2)  | \
                        (drv8316SPIVars->Ctrl_Reg_07.EN_AAR        << 3)  | \
                        (drv8316SPIVars->Ctrl_Reg_07.AD_COMP_TH_HS << 4)  | \
                        (drv8316SPIVars->Ctrl_Reg_07.AD_COMP_TH_LS << 5)  | \
                        (drv8316SPIVars->Ctrl_Reg_07.ILIM_RECIR    << 6)  | \
                        (drv8316SPIVars->Ctrl_Reg_07.BEMF_TH       << 7);
           DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

       // Write Control Register 6
          drvRegAddr = DRV8316_ADDRESS_CONTROL_6;
          drvDataNew = (drv8316SPIVars->Ctrl_Reg_08.BUCK_DIS    << 0)  | \
                       (drv8316SPIVars->Ctrl_Reg_08.BUCK_SEL)          | \
                       (drv8316SPIVars->Ctrl_Reg_08.BUCK_CL     << 3)  | \
                       (drv8316SPIVars->Ctrl_Reg_08.BUCK_PS_DIS << 4)  | \
                       (drv8316SPIVars->Ctrl_Reg_08.BUCK_SR     << 5)  | \
                       (drv8316SPIVars->Ctrl_Reg_08.CTRL08_RSV1 << 6)  | \
                       (drv8316SPIVars->Ctrl_Reg_08.CTRL08_RSV2 << 7);
          DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8316SPIVars->writeCmd = false;
    }

    // Manual write to the DRV8316
    if(drv8316SPIVars->manWriteCmd)
    {
        // Custom Write
        drvRegAddr = (DRV8316_Address_e)(drv8316SPIVars->manWriteAddr << 11);
        drvDataNew = drv8316SPIVars->manWriteData;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8316SPIVars->manWriteCmd = false;
    }

    return;
}  // end of DRV8316_writeData() function

void DRV8316_readData(DRV8316_Handle handle, DRV8316_SPIVars_t *drv8316SPIVars)
{
    DRV8316_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8316SPIVars->readCmd)
    {
        // Read registers for default values
        // Read Status Register 0
        drvRegAddr = DRV8316_ADDRESS_IC_STAT_0;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->IC_Stat_Reg_00.FAULT         = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_FAULT_BITS)?1:0;
        drv8316SPIVars->IC_Stat_Reg_00.OT            = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_OT_BITS)?1:0;
        drv8316SPIVars->IC_Stat_Reg_00.OVP           = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_OVP_BITS)?1:0;
        drv8316SPIVars->IC_Stat_Reg_00.NPOR          = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_NPOR_BITS)?1:0;
        drv8316SPIVars->IC_Stat_Reg_00.SPI_FLT       = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_SPI_FLT_BITS)?1:0;
        drv8316SPIVars->IC_Stat_Reg_00.BK_FLT        = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_BK_FLT_BITS)?1:0;
        drv8316SPIVars->IC_Stat_Reg_00.IC_STAT_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8316_IC_STAT00_RESERVED_BITS)?1:0;

        // Read Status Register 1
        drvRegAddr = DRV8316_ADDRESS_STATUS_1;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Stat_Reg_01.OCP_LA  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_LA_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_01.OCP_HA  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_HA_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_01.OCP_LB  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_LB_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_01.OCP_HB  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_HB_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_01.OCP_LC  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_LC_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_01.OCP_HC  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OCP_HC_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_01.OTS     = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OTS_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_01.OTW     = (bool)(drvDataNew & (uint16_t)DRV8316_STAT01_OTW_BITS)?1:0;

        // Read Status Register 2
        drvRegAddr = DRV8316_ADDRESS_STATUS_2;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Stat_Reg_02.SPI_ADDR_FLT  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_SPI_ADDR_FLT_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_02.SPI_SCLK_FLT  = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_SPI_SCLK_FLT_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_02.SPI_PARITY    = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_SPI_PARITY_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_02.VCP_UV        = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_VCP_UV_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_02.BUCK_UV       = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_BUCK_UV_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_02.BUCK_OCP      = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_BUCK_OCP_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_02.OTP_ERR       = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_OTP_ERR_BITS)?1:0;
        drv8316SPIVars->Stat_Reg_02.STAT02_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8316_STAT02_RESERVED_BITS)?1:0;

        // Read Control Register 1
        drvRegAddr = DRV8316_ADDRESS_CONTROL_1;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Ctrl_Reg_03.REG_LOCK    = (DRV8316_CTRL03_RegLock_e)(drvDataNew & (uint16_t)DRV8316_CTRL03_REG_LOCK_BITS);
        drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED1_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED2_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED3_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED4_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_03.CTRL03_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL03_RESERVED5_BITS)?1:0;

        // Read Control Register 2
        drvRegAddr = DRV8316_ADDRESS_CONTROL_2;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Ctrl_Reg_04.CLR_FLT      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_CLR_FLT_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_04.PWM_MODE     = (DRV8316_CTRL04_PwmMode_e)(drvDataNew & (uint16_t)DRV8316_CTRL04_PWM_MODE_BITS);
        drv8316SPIVars->Ctrl_Reg_04.SLEW         = (DRV8316_CTRL04_SlewRate_e)(drvDataNew & (uint16_t)DRV8316_CTRL04_SLEW_BITS);
        drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_RESERVED1_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_RESERVED2_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_04.CTRL04_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL04_RESERVED3_BITS)?1:0;

        // Read Control Register 3
        drvRegAddr = DRV8316_ADDRESS_CONTROL_3;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Ctrl_Reg_05.OTW_REP      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_OTW_REP_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_05.SPI_FLT_REP  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_SPI_FLT_REP_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_05.OVP_EN       = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_OVP_EN_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_05.OVP_SEL      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_OVP_SEL_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED1_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED2_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED3_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_05.CTRL05_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL05_RESERVED4_BITS)?1:0;

        // Read Control Register 4
        drvRegAddr = DRV8316_ADDRESS_CONTROL_4;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Ctrl_Reg_06.OCP_MODE     = (DRV8316_CTRL06_OcpMode_e)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_MODE_BITS);
        drv8316SPIVars->Ctrl_Reg_06.OCP_LVL      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_LVL_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_06.OCP_RETRY    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_RETRY_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_06.OCP_DEG      = (DRV8316_CTRL06_OcpDeglitch_e)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_DEG_BITS);
        drv8316SPIVars->Ctrl_Reg_06.OCP_CBC      = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_OCP_CBC_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_06.CTRL06_RSV   = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL06_RESERVED_BITS)?1:0;

          // Read Control Register 5
        drvRegAddr = DRV8316_ADDRESS_CONTROL_5;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Ctrl_Reg_07.CSA_GAIN       = (DRV8316_CTRL07_CsaGain_e)(drvDataNew & (uint16_t)DRV8316_CTRL07_CSA_GAIN_BITS);
        drv8316SPIVars->Ctrl_Reg_07.EN_ASR         = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_EN_ASR_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_07.EN_AAR         = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_EN_AAR_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_07.AD_COMP_TH_HS  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_AD_COMP_TH_HS_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_07.AD_COMP_TH_LS  = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_AD_COMP_TH_LS_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_07.ILIM_RECIR     = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_ILIM_RECIR_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_07.BEMF_TH        = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL07_BEMF_TH_BITS)?1:0;

        // Read Control Register 6
        drvRegAddr = DRV8316_ADDRESS_CONTROL_6;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->Ctrl_Reg_08.BUCK_DIS       = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_DIS_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_08.BUCK_SEL       = (DRV8316_CTRL08_BuckSel_e)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_SEL_BITS);
        drv8316SPIVars->Ctrl_Reg_08.BUCK_CL        = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_CL_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_08.BUCK_PS_DIS    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_PS_DIS_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_08.BUCK_SR        = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_BUCK_SR_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_08.CTRL08_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_RESERVED1_BITS)?1:0;
        drv8316SPIVars->Ctrl_Reg_08.CTRL08_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8316_CTRL08_RESERVED2_BITS)?1:0;

        drv8316SPIVars->readCmd = false;
    }

    // Manual read from the DRV8316
    if(drv8316SPIVars->manReadCmd)
    {
        // Custom Read
        drvRegAddr = (DRV8316_Address_e)(drv8316SPIVars->manReadAddr << 11);
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316SPIVars->manReadData = drvDataNew;

        drv8316SPIVars->manReadCmd = false;
    }

    return;
}  // end of DRV8316_readData() function

// end of file
