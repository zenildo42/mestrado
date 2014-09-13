/*
 * Copyright 2013 OpenMote Technologies, S.L.
 */

/**
 *
 * @file       Uart.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2014
 * @brief
 * @ingroup
 *
 */

/*================================ include ==================================*/

#include "Uart.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Uart::Uart(uint32_t peripheral_, uint32_t base_, uint32_t clock_, uint32_t interrupt_, GpioUart& rx_, GpioUart& tx_):
    peripheral(peripheral_), base(base_), clock(clock_), interrupt(interrupt_), rx(rx_), tx(tx_)
{
}

uint32_t Uart::getBase(void)
{
    return base;
}

void Uart::enable(uint32_t baudrate_, uint32_t config_, uint32_t mode_)
{
    // Store the UART baudrate, configuration and mode
    baudrate = baudrate_;
    config   = config_;
    mode     = mode_;

    // Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3)
    SysCtrlPeripheralEnable(peripheral);
    SysCtrlPeripheralSleepEnable(peripheral);
    SysCtrlPeripheralDeepSleepDisable(peripheral);

    // Disable peripheral previous to configuring it
    UARTDisable(peripheral);

    // Set IO clock as UART clock source
    UARTClockSourceSet(base, clock);

    // Configure the UART RX and TX pins
    IOCPinConfigPeriphInput(rx.getPort(), rx.getPin(), rx.getIoc());
    IOCPinConfigPeriphOutput(tx.getPort(), tx.getPin(), tx.getIoc());

    // Configure the UART GPIOs
    GPIOPinTypeUARTInput(rx.getPort(), rx.getPin());
    GPIOPinTypeUARTOutput(tx.getPort(), tx.getPin());

    // Configure the UART
    UARTConfigSetExpClk(base, SysCtrlIOClockGet(), baudrate, config);

    // Disable FIFO as we only use a one-byte buffer
    UARTFIFODisable(base);

    // Raise an interrupt at the end of transmission
    UARTTxIntModeSet(base, mode);

    // Enable UART hardware
    UARTEnable(base);
}

void Uart::sleep(void)
{
    // Wait until UART is not busy
    while(UARTBusy(base))
        ;

    // Disable UART hardware
    UARTDisable(base);

    // Configure the pins as outputs
    GPIOPinTypeGPIOOutput(rx.getPort(), rx.getPin());
    GPIOPinTypeGPIOOutput(tx.getPort(), tx.getPin());

    // Pull the pins to ground
    GPIOPinWrite(rx.getPort(), rx.getPin(), 0);
    GPIOPinWrite(tx.getPort(), tx.getPin(), 0);
}

void Uart::wakeup(void)
{
    // Re-enable the UART interface
    enable(baudrate, config, mode);
}

void Uart::setRxCallback(Callback* callback_)
{
    rx_callback = callback_;
}

void Uart::setTxCallback(Callback* callback_)
{
    tx_callback = callback_;
}

void Uart::enableInterrupt(void)
{
    // Register the interrupt handler
    InterruptHandler::getInstance().setInterruptHandler(this);

    // Enable the UART RX, TX and RX timeout interrupts
    UARTIntEnable(base, UART_INT_RX | UART_INT_TX | UART_INT_RT);

    // Set the UART interrupt priority
    IntPrioritySet(interrupt, (7 << 5));

    // Enable the UART interrupt
    IntEnable(interrupt);
}

void Uart::disableInterrupt(void)
{
    // Disable the UART RX, TX and RX timeout interrupts
    UARTIntDisable(base, UART_INT_RX | UART_INT_TX | UART_INT_RT);

    // Disable the UART interrupt
    IntDisable(interrupt);
}

uint8_t Uart::readByte(void)
{
    int32_t byte;
    byte = UARTCharGetNonBlocking(base);
    return (uint8_t)(byte & 0xFF);
}

uint32_t Uart::readByte(uint8_t * buffer, uint32_t length)
{
    uint32_t data;
    for (uint32_t i = 0; i < length; i++)
    {
        data = UARTCharGet(base);
        *buffer++ = (uint8_t) data;
    }

    // Wait until it is complete
    while(UARTBusy(base))
        ;

    return 0;
}

void Uart::writeByte(uint8_t byte)
{
    UARTCharPutNonBlocking(base, byte);
}

uint32_t Uart::writeByte(uint8_t * buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        UARTCharPut(base, *buffer++);
    }

    // Wait until it is complete
    while(UARTBusy(base))
        ;

    return 0;
}

/*=============================== protected =================================*/

void Uart::interruptHandler(void)
{
    uint32_t status;

    // Read interrupt source
    status = UARTIntStatus(base, true);

    // Clear UART interrupt in the NVIC
    IntPendClear(interrupt);

    // Process TX interrupt
    if (status & UART_INT_TX)
    {
        UARTIntClear(base, UART_INT_TX);
        interruptHandlerTx();
    }

    // Process RX interrupt
    if (status & UART_INT_RX ||
        status & UART_INT_RT)
    {
        UARTIntClear(base, UART_INT_RX | UART_INT_RT);
        interruptHandlerRx();
    }
}

/*================================ private ==================================*/

void Uart::interruptHandlerRx(void)
{
    if (rx_callback != nullptr)
    {
        rx_callback->execute();
    }
}

void Uart::interruptHandlerTx(void)
{
    if (tx_callback != nullptr)
    {
        tx_callback->execute();
    }
}
