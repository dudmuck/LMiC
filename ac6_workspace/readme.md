setup sw4stm32 (ac6 openstm32):

Assign location of STM32 HAL peripheral driver:
    Project -> Properties:
        C/C++ Build -> Environment:
            Add variable STM32_HAL, set to location of your root directory of STM32Cube_FW_Lx_Vx.x.x
        Resource -> Linked Resources:
            add variable STM32HAL, set to STM32Cube_FW_Lx_Vx.x.x/Drivers/STM32Lxxx_HAL_Driver
