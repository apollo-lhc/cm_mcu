/*
 * I2CMasterTask.c
 *
 *  Created on: Feb 7, 2020
 *      Author: wittich
 */

/* Gakekeeper task for I2C Slave */

#include "I2CMasterTask.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "common/smbus.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"

//*****************************************************************************
//
// The states for the master and slave interrupt handler state machines.
//
//*****************************************************************************
#define SMBUS_STATE_IDLE                0
#define SMBUS_STATE_SLAVE_POST_COMMAND  1
#define SMBUS_STATE_WRITE_BLOCK_SIZE    2
#define SMBUS_STATE_WRITE_NEXT          3
#define SMBUS_STATE_WRITE_FINAL         4
#define SMBUS_STATE_WRITE_DONE          5
#define SMBUS_STATE_READ_ONE            6
#define SMBUS_STATE_READ_FIRST          7
#define SMBUS_STATE_READ_BLOCK_SIZE     8
#define SMBUS_STATE_READ_NEXT           9
#define SMBUS_STATE_READ_FINAL          10
#define SMBUS_STATE_READ_WAIT           11
#define SMBUS_STATE_READ_PEC            12
#define SMBUS_STATE_READ_DONE           13
#define SMBUS_STATE_READ_ERROR_STOP     14

//*****************************************************************************
//
// Status flags for various instance-specific tasks.
//
//*****************************************************************************
#define FLAG_PEC                        0
#define FLAG_PROCESS_CALL               1
#define FLAG_BLOCK_TRANSFER             2
#define FLAG_TRANSFER_IN_PROGRESS       3
#define FLAG_RAW_I2C                    4
#define FLAG_ADDRESS_RESOLVED           5
#define FLAG_ADDRESS_VALID              6
#define FLAG_ARP                        7


static
tSMBusStatus
ProcessSMBusInterrupt(tSMBus *psSMBus)
{
 // uint32_t ui32IntStatus;
  uint32_t ui32ErrorStatus;
  uint8_t ui8TempData;

  //
  // Read the master interrupt status bits.
  //
  ui32ErrorStatus = HWREG(psSMBus->ui32I2CBase + I2C_O_MCS);

  //
  // Check for arbitration lost.
  //
  if(ui32ErrorStatus & I2C_MCS_ARBLST)
  {
    //
    // Put the state machine back in the idle state.
    //
    psSMBus->ui8MasterState = SMBUS_STATE_IDLE;

    //
    // Clear the transfer in progress flag.
    //
    HWREGBITB(&psSMBus->ui16Flags, FLAG_TRANSFER_IN_PROGRESS) = 0;

    //
    // Return to caller.
    //
    return(SMBUS_ARB_LOST);
  }

  //
  // Check for an error.
  //
  if(ui32ErrorStatus & I2C_MCS_ERROR)
  {
    //
    // Put the state machine back in the idle state.
    //
    psSMBus->ui8MasterState = SMBUS_STATE_IDLE;

    //
    // Check to see if the bus is free.  There are two interrupts when a
    // NACK happens, and the bus should only be free during the second
    // interrupt.  During the first interrupt (when the bus is busy),
    // generate the necessary STOP condition.
    //
    if(MAP_I2CMasterBusBusy(psSMBus->ui32I2CBase))
    {
      //
      // Issue a STOP.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
          I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    }
    else
    {
      //
      // Clear the transfer in progress flag.
      //
      HWREGBITB(&psSMBus->ui16Flags, FLAG_TRANSFER_IN_PROGRESS) = 0;
    }

    //
    // Check for ACK errors.
    //
    if(ui32ErrorStatus & I2C_MCS_ADRACK)
    {
      //
      // Return to caller.
      //
      return(SMBUS_ADDR_ACK_ERROR);
    }
    else if(ui32ErrorStatus & I2C_MCS_DATACK)
    {
      //
      // Return to caller.
      //
      return(SMBUS_DATA_ACK_ERROR);
    }
    else
    {
      //
      // Return to caller.  Should never get here.
      //
      return(SMBUS_MASTER_ERROR);
    }
  }

  //
  // If no error conditions, determine what to do based on the state.
  //
  switch(psSMBus->ui8MasterState)
  {
  //
  // The idle state.  This state should only be reached after the last
  // byte of a master transmit.
  //
  case SMBUS_STATE_IDLE:
  {
    //
    // If the peripheral is not busy clear the transfer in progress
    // flag.  This means that the peripheral has given up the bus,
    // most likely due to the end of a transmit operation.
    //
    if(!MAP_I2CMasterBusy(psSMBus->ui32I2CBase))
    {
      //
      // Clear the transfer in progress flag.
      //
      HWREGBITB(&psSMBus->ui16Flags, FLAG_TRANSFER_IN_PROGRESS) = 0;
    }

    //
    // This state is done.
    //
    break;
  }

  //
  // When using a block write, the transfer size must be sent before the
  // data payload.
  //
  case SMBUS_STATE_WRITE_BLOCK_SIZE:
  {
    //
    // Write the block write size to the data register.
    //
    MAP_I2CMasterDataPut(psSMBus->ui32I2CBase, psSMBus->ui8TxSize);

    //
    // Continue the burst write.
    //
    MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                         I2C_MASTER_CMD_BURST_SEND_CONT);

    //
    // The next data byte is from the data payload.
    //
    if((psSMBus->ui8TxSize == 1) &&
        !(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC)))
    {
      psSMBus->ui8MasterState = SMBUS_STATE_WRITE_FINAL;
    }
    else
    {
      psSMBus->ui8MasterState = SMBUS_STATE_WRITE_NEXT;
    }

    //
    // This state is done.
    //
    break;
  }
  //
  // The state for the middle of a burst write.
  //
  case SMBUS_STATE_WRITE_NEXT:
  {
    //
    // Write the next byte to the data register.
    //
    MAP_I2CMasterDataPut(psSMBus->ui32I2CBase,
        psSMBus->pui8TxBuffer[psSMBus->ui8TxIndex++]);

    //
    // Continue the burst write.
    //
    MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                         I2C_MASTER_CMD_BURST_SEND_CONT);

    //
    // Determine the next state based on the values of the PEC and
    // process call flags.
    //

    //
    // If PEC is active and process call is not active.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // If a process call, there is no PEC byte on the transmit.
      //
      if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PROCESS_CALL))
      {
        //
        // Check to see if the TX index is equal to size minus 1.
        //
        if(psSMBus->ui8TxIndex == (psSMBus->ui8TxSize - 1))
        {
          psSMBus->ui8MasterState = SMBUS_STATE_WRITE_FINAL;
        }
      }
      else
      {
        //
        // If the TX index is the same as the size, we're done.
        //
        if(psSMBus->ui8TxIndex == psSMBus->ui8TxSize)
        {
          psSMBus->ui8MasterState = SMBUS_STATE_WRITE_FINAL;
        }
      }
    }

    //
    // If PEC is not used, regardless of whether this is a process
    // call.
    //
    else
    {
      //
      // Check to see if the TX index is equal to the size minus 1.
      //
      if(psSMBus->ui8TxIndex == (psSMBus->ui8TxSize - 1))
      {
        psSMBus->ui8MasterState = SMBUS_STATE_WRITE_FINAL;
      }
    }

    //
    // This state is done.
    //
    break;
  }

  //
  // The state for the final write of a burst sequence.
  //
  case SMBUS_STATE_WRITE_FINAL:
  {
    //
    // Determine what data to write to the data register based
    // on the values of the PEC and process call flags.
    //
    //
    // If PEC is active, write the PEC byte to the data register.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // If a process call is active, send data, not CRC.
      //
      if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PROCESS_CALL))
      {
        //
        // Write the final byte from TX buffer to the data
        // register.
        //
        MAP_I2CMasterDataPut(psSMBus->ui32I2CBase,
            psSMBus->pui8TxBuffer[psSMBus->
                                  ui8TxIndex++]);
      }
      else
      {
        //
        // Write the calculated CRC (PEC) byte to the data
        // register.
        //
        MAP_I2CMasterDataPut(psSMBus->ui32I2CBase,
            psSMBus->ui8CalculatedCRC);
      }
    }
    else
    {
      //
      // Write the final byte from TX buffer to the data register.
      //
      MAP_I2CMasterDataPut(psSMBus->ui32I2CBase,
          psSMBus->pui8TxBuffer[psSMBus->
                                ui8TxIndex++]);
    }

    //
    // If a process call is active, send out the repeated start to
    // begin the RX portion.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PROCESS_CALL))
    {
      //
      // Move to the read first "turnaround" state.
      //
      psSMBus->ui8MasterState = SMBUS_STATE_READ_FIRST;

      //
      // Continue the burst write.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                           I2C_MASTER_CMD_BURST_SEND_CONT);
    }
    else
    {
      //
      // Finish the burst write.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
          I2C_MASTER_CMD_BURST_SEND_FINISH);

      //
      // Since we end the transaction after the last byte is sent,
      // the next state is idle.
      //
      psSMBus->ui8MasterState = SMBUS_STATE_IDLE;
    }

    //
    // This state is done.
    //
    break;
  }

  //
  // The state for a single byte read.
  //
  case SMBUS_STATE_READ_ONE:
  {
    //
    // Put the I2C master into receive mode.
    //
    MAP_I2CMasterSlaveAddrSet(psSMBus->ui32I2CBase,
        psSMBus->ui8TargetSlaveAddress, true);

    //
    // Perform a single byte read.
    //
    MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                         I2C_MASTER_CMD_SINGLE_RECEIVE);

    //
    // The next state is the wait for final read state.
    //
    psSMBus->ui8MasterState = SMBUS_STATE_READ_WAIT;

    //
    // This state is done.
    //
    break;
  }

  //
  // The state for the start of a burst read.
  //
  case SMBUS_STATE_READ_FIRST:
  {
    //
    // Put the I2C master into receive mode.
    //
    MAP_I2CMasterSlaveAddrSet(psSMBus->ui32I2CBase,
        psSMBus->ui8TargetSlaveAddress, true);

    //
    // Handle the case where PEC is used.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // Add the target address and R/S bit to the running CRC
      // calculation.
      //
      ui8TempData =
          ((psSMBus->ui8TargetSlaveAddress << 1) & 0xfe) | 1;

      //
      // Update the calculated CRC value in the configuration
      // structure.
      //
      psSMBus->ui8CalculatedCRC =
          MAP_Crc8CCITT(psSMBus->ui8CalculatedCRC, &ui8TempData, 1);

      //
      // Set the next state in the state machine.
      //
      if(psSMBus->ui8RxSize > 1)
      {
        //
        // If this is a block transfer, the next state is to read
        // back the number of bytes that the slave will be sending.
        //
        if(HWREGBITB(&psSMBus->ui16Flags, FLAG_BLOCK_TRANSFER))
        {
          psSMBus->ui8MasterState = SMBUS_STATE_READ_BLOCK_SIZE;
        }

        //
        // For every other case...
        //
        else
        {
          psSMBus->ui8MasterState = SMBUS_STATE_READ_NEXT;
        }
      }

      //
      // If 1 byte remains, move to the final read state.
      //
      else
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_FINAL;
      }
    }
    else
    {
      //
      // Set the next state in the state machine.
      //
      if(psSMBus->ui8RxSize > 2)
      {
        //
        // If this is a block transfer, the next state is to read
        // back the number of bytes that the slave will be sending.
        //
        if(HWREGBITB(&psSMBus->ui16Flags, FLAG_BLOCK_TRANSFER))
        {
          psSMBus->ui8MasterState = SMBUS_STATE_READ_BLOCK_SIZE;
        }

        //
        // For every other case...
        //
        else
        {
          psSMBus->ui8MasterState = SMBUS_STATE_READ_NEXT;
        }
      }

      //
      // If 2 bytes remain, move to the final read state.
      //
      else
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_FINAL;
      }
    }

    //
    // Start the burst receive.
    //
    MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                         I2C_MASTER_CMD_BURST_RECEIVE_START);

    //
    // This state is done.
    //
    break;
  }

  //
  // The state for the size of a block read.
  //
  case SMBUS_STATE_READ_BLOCK_SIZE:
  {
    //
    // Update the RX size with the data byte.
    //
    psSMBus->ui8RxSize = MAP_I2CMasterDataGet(psSMBus->ui32I2CBase);

    //
    // If more than 32 bytes are going to be sent, error.
    //
    if((psSMBus->ui8RxSize > 32) || (psSMBus->ui8RxSize == 0))
    {
      //
      // Set the next state.
      //
      psSMBus->ui8MasterState = SMBUS_STATE_READ_ERROR_STOP;

      //
      // If too many or too few bytes, error.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                           I2C_MASTER_CMD_SINGLE_RECEIVE);

      //
      // Break from this case.
      //
      break;
    }

    //
    // If PEC is enabled, add the size byte to the calculation and
    // add one to the size variable to account for the extra PEC byte.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // Calculate the new CRC and update configuration structure.
      //
      psSMBus->ui8CalculatedCRC =
          MAP_Crc8CCITT(psSMBus->ui8CalculatedCRC,
              &psSMBus->ui8RxSize, 1);
    }

    //
    // Update the state machine.
    //
    switch(psSMBus->ui8RxSize)
    {
    //
    // 1 byte remaining.
    //
    case 1:
    {
      //
      // If only one byte remains and PEC, go to the second
      // to last byte state.
      //
      if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_FINAL;
      }

      //
      // If only one byte remains and no PEC, end the burst
      // transfer.
      //
      else
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_WAIT;
      }

      //
      // This switch is done.
      //
      break;
    }

    //
    // 2 bytes remaining.
    //
    case 2:
    {
      //
      // If two bytes and PEC remain, move to read next
      // state.
      //
      if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_NEXT;
      }

      //
      // If two bytes remain, move to the final read state.
      //
      else
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_FINAL;
      }

      //
      // This switch is done.
      //
      break;
    }

    //
    // For every other situation (in other words, remaining bytes
    // is greater than 2).
    //
    default:
    {
      //
      // If more than 2 bytes to read, move to the next byte
      // state.
      //
      psSMBus->ui8MasterState = SMBUS_STATE_READ_NEXT;

      //
      // This switch is done.
      //
      break;
    }
    }

    //
    // Determine how to step the I2C state machine.
    //
    if((psSMBus->ui8RxSize == 1) &&
        !HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // If exactly 1 byte remains, read the byte and send a STOP.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
          I2C_MASTER_CMD_BURST_SEND_FINISH);
    }
    else
    {
      //
      // Otherwise, continue the burst read.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
          I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    }

    //
    // This state is done.
    //
    break;
  }

  //
  // The state for the middle of a burst read.
  //
  case SMBUS_STATE_READ_NEXT:
  {
    //
    // Check for a buffer overrun.
    //
    if(psSMBus->ui8RxIndex >= psSMBus->ui8RxSize)
    {
      //
      // Dummy read of data register.
      //
      ui8TempData = MAP_I2CMasterDataGet(psSMBus->ui32I2CBase);

      //
      // If too many or too few bytes, error.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                           I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

      //
      // Set the next state.
      //
      psSMBus->ui8MasterState = SMBUS_STATE_READ_ERROR_STOP;

      //
      // Break from this case.
      //
      break;
    }

    //
    // Read the received character.
    //
    psSMBus->pui8RxBuffer[psSMBus->ui8RxIndex] =
        MAP_I2CMasterDataGet(psSMBus->ui32I2CBase);

    //
    // Continue the burst read.
    //
    MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                         I2C_MASTER_CMD_BURST_RECEIVE_CONT);

    //
    // If PEC is enabled, add the received byte to the calculation.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // Calculate the new CRC and update configuration structure.
      //
      psSMBus->ui8CalculatedCRC =
          MAP_Crc8CCITT(psSMBus->ui8CalculatedCRC,
              &psSMBus->pui8RxBuffer[psSMBus->ui8RxIndex],
              1);

      //
      // Increment the receive buffer index.
      //
      psSMBus->ui8RxIndex++;

      //
      // If there is 1 byte remaining, make next state be the
      // end of burst read state.
      //
      if((psSMBus->ui8RxSize - psSMBus->ui8RxIndex) == 1)
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_FINAL;
      }
    }
    else
    {
      //
      // Increment the receive buffer index.
      //
      psSMBus->ui8RxIndex++;

      //
      // If there are two bytes remaining, make next state be the
      // end of burst read state.
      //
      if((psSMBus->ui8RxSize - psSMBus->ui8RxIndex) == 2)
      {
        psSMBus->ui8MasterState = SMBUS_STATE_READ_FINAL;
      }
    }

    //
    // This state is done.
    //
    break;
  }

  //
  // The state for the end of a burst read.
  //
  case SMBUS_STATE_READ_FINAL:
  {
    //
    // Check for a buffer overrun.
    //
    if(psSMBus->ui8RxIndex >= psSMBus->ui8RxSize)
    {
      //
      // Dummy read of data register.
      //
      ui8TempData = MAP_I2CMasterDataGet(psSMBus->ui32I2CBase);

      //
      // If too many or too few bytes, error.
      //
      MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                           I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

      //
      // Set the next state.
      //
      psSMBus->ui8MasterState = SMBUS_STATE_READ_ERROR_STOP;

      //
      // Break from this case.
      //
      break;
    }

    //
    // Read the received character.
    //
    psSMBus->pui8RxBuffer[psSMBus->ui8RxIndex] =
        MAP_I2CMasterDataGet(psSMBus->ui32I2CBase);

    //
    // The next state is the wait for final read state.
    //
    psSMBus->ui8MasterState = SMBUS_STATE_READ_WAIT;

    //
    // Finish the burst read.
    //
    MAP_I2CMasterControl(psSMBus->ui32I2CBase,
                         I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    //
    // If PEC is enabled, add the received byte to the calculation.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // Calculate the new CRC and update configuration structure.
      //
      psSMBus->ui8CalculatedCRC =
          MAP_Crc8CCITT(psSMBus->ui8CalculatedCRC,
              &psSMBus->pui8RxBuffer[psSMBus->ui8RxIndex],
              1);
    }

    //
    // Increment the receive buffer index.
    //
    psSMBus->ui8RxIndex++;

    //
    // This state is done.
    //
    break;
  }

  //
  // This state is for the final read of a single or burst read.
  //
  case SMBUS_STATE_READ_WAIT:
  {
    //
    // Read the received byte.
    //
    ui8TempData = MAP_I2CMasterDataGet(psSMBus->ui32I2CBase);

    //
    // If PEC is enabled, check the value that just came in to see
    // if it matches.
    //
    if(HWREGBITB(&psSMBus->ui16Flags, FLAG_PEC))
    {
      //
      // Check for a buffer overrun.
      //
      if(psSMBus->ui8RxIndex > psSMBus->ui8RxSize)
      {
        //
        // Clear the transfer in progress flag.
        //
        HWREGBITB(&psSMBus->ui16Flags,
            FLAG_TRANSFER_IN_PROGRESS) = 0;

        //
        // Return the error condition.
        //
        return(SMBUS_DATA_SIZE_ERROR);
      }

      //
      // Store the received CRC byte.
      //
      psSMBus->ui8ReceivedCRC = ui8TempData;

      //
      // If the CRC doesn't match, send a NACK and indicate the
      // failure to the application.
      //
      if(psSMBus->ui8ReceivedCRC != psSMBus->ui8CalculatedCRC)
      {
        //
        // Clear the transfer in progress flag.
        //
        HWREGBITB(&psSMBus->ui16Flags,
            FLAG_TRANSFER_IN_PROGRESS) = 0;

        //
        // Return the error condition.
        //
        return(SMBUS_PEC_ERROR);
      }
    }
    else
    {
      //
      // Check for a buffer overrun.
      //
      if(psSMBus->ui8RxIndex >= psSMBus->ui8RxSize)
      {
        //
        // Clear the transfer in progress flag.
        //
        HWREGBITB(&psSMBus->ui16Flags,
            FLAG_TRANSFER_IN_PROGRESS) = 0;

        //
        // Return the error condition.
        //
        return(SMBUS_DATA_SIZE_ERROR);
      }

      //
      // Read the received byte.
      //
      psSMBus->pui8RxBuffer[psSMBus->ui8RxIndex] = ui8TempData;

      //
      // Increment the receive buffer index.
      //
      psSMBus->ui8RxIndex++;
    }

    //
    // The state machine is now idle.
    //
    psSMBus->ui8MasterState = SMBUS_STATE_IDLE;

    //
    // Clear the transfer in progress flag.
    //
    HWREGBITB(&psSMBus->ui16Flags, FLAG_TRANSFER_IN_PROGRESS) = 0;

    //
    // This state is done.
    //
    break;
  }

  //
  // This state is for a transaction that needed to end due to a
  // size error.
  //
  case SMBUS_STATE_READ_ERROR_STOP:
  {
    //
    // Dummy read the received byte.
    //
    ui8TempData = MAP_I2CMasterDataGet(psSMBus->ui32I2CBase);

    //
    // The state machine is now idle.
    //
    psSMBus->ui8MasterState = SMBUS_STATE_IDLE;

    //
    // Clear the transfer in progress flag.
    //
    HWREGBITB(&psSMBus->ui16Flags, FLAG_TRANSFER_IN_PROGRESS) = 0;

    //
    // Return the error condition.
    //
    return(SMBUS_DATA_SIZE_ERROR);
  }
  }

  //
  // Return to caller.
  //
  return(SMBUS_OK);
}


void I2CMasterTask(void *parameters)
{
  TaskNotifyI2CMaster = xTaskGetCurrentTaskHandle();

  struct I2CMasterTaskArgs_t *args = parameters;


  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    // wait for something from the interrupt handler
    unsigned long ulNotificationValue = xTaskNotifyWait(0UL,0xFFFFFFFFUL,
        &interruptStatus, xTicksToWait);
    if ( ulNotificationValue == pdFALSE ) {
      theState = I2C_READY;
      addr = 0U;
      val = 0U;
      continue;
    }
    TaskNotifyI2CMaster = xTaskGetCurrentTaskHandle();


    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  } // infinite loop for task
}

