# *Stm32-F407-FATFS SubProgram*

This program is intended to understand the process of FATFS including `f_mount()`,`f_open()`,`f_close()`,`f_read()`,
`f_write()`. It does not sound difficult, so I try to add a FreeRTOS working environment and a User Interface which is
done by the UART transmission process in DMA mode.

## How to use
1. Clion\MinGW\OpenOCD\arm-none-eabi-gcc
2. ST-link\Stm32F407
3. Download and run!(Try not to create new files in .ioc since the program has been modified somewhere hidden)

## Functionality
This subprogram implement such a functionality:

- List all files currently 
- Create a new file 
- Create a new directory 
- Delete a existed file 
- Modify a file (Enter 'q/Q' to quit)
- Read a file


## Detail
The board I'm using has something wrong with its PLL config register. It can not initialize when the clock source is
chosen to be PLL. *SO I have to use a 8MHz HSE clock*, but it should be fine when using another new board.

The UART is configured the same as in Stm32-UART_DMA Project:
- PA9: Tx___PA10-Rx
- BaudRate = 115200 baud
- Word Length = 8 bits
- One Stop bit
- None parity
- Hardware flow control disabled
- Rx and Tx are enabled at the same time

The SDIO is configured just:
- Mode: SD 4bits Wide bus.
- PC8 ___ SDIO_D0
- PC9 ___ SDIO_D1
- PC10___ SDIO_D2
- PC11___ SDIO_D3
- PC12___ SDIO_CK
- PD2 ___ SDIO_CMD
- The SDIO_RX and SDIO_TX DMA channel should open as DMA2 Stream3 and DMA2 Stream6( *4 bytes alignment automatically*)

### Take care
1. The `__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);` must be added after the CubeMX automatically generates the code.
2. A `USER_UART_IRQHandler(UART_HandleTypeDef* huart)` should be defined and added to the `UART1_IRQHandler` to answer 
the UART_IDLE interrupt. (*The above two actions are identifying whether we receive a message through UART, When it does,
it goes to the UART_IT_IDLE*)
3. Every time after a reception process, the DMA should be reinit to set the memory inc pointer to its original place. Or the data mat be displacement.
4. When using FATFS, the heap and stack should be big enough. Both in the setting of your IDE and FREERTOS config file.
In my project, I set heap and stack 0x2000 each in CubeMX and each FreeRTOS task with 1024 Word. Although I only use *one*
task. If not, The program would somehow goes into a HardFault_Handler.
