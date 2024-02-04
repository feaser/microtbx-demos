# Create interface library for the STM32 CubeF4 HAL sources.
add_library(cube_f4_hal INTERFACE)

target_sources(cube_f4_hal INTERFACE
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_adc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_adc_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_can.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_cec.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_cortex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_crc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_cryp.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_cryp_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dac.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dac_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dcmi.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dcmi_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dfsdm.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dma2d.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dma.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dma_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_dsi.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_eth.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_exti.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_flash.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_flash_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_flash_ramfunc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_fmpi2c.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_fmpi2c_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_fmpsmbus.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_fmpsmbus_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_gpio.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_hash.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_hash_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_hcd.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_i2c.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_i2c_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_i2s.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_i2s_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_irda.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_iwdg.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_lptim.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_ltdc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_ltdc_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_mmc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_nand.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_nor.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_pccard.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_pcd.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_pcd_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_pwr.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_pwr_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_qspi.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_rcc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_rcc_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_rng.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_rtc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_rtc_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_sai.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_sai_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_sd.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_sdram.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_smartcard.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_smbus.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_spdifrx.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_spi.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_sram.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_tim.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_tim_ex.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_uart.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_usart.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_hal_wwdg.c"
)

target_include_directories(cube_f4_hal INTERFACE 
    "${CMAKE_SOURCE_DIR}/third_party/cmsis/core/CMSIS/Core/Include"
    "${CMAKE_SOURCE_DIR}/third_party/cmsis/device/st/f4/Include"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Inc"
)

target_compile_definitions(cube_f4_hal INTERFACE
    USE_HAL_DRIVER
)

# Create interface library for the STM32 CubeF4 LL sources.
add_library(cube_f4_ll INTERFACE)

target_sources(cube_f4_ll INTERFACE
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_adc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_crc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_dac.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_dma2d.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_dma.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_exti.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_fmc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_fmpi2c.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_fsmc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_gpio.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_i2c.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_lptim.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_pwr.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_rcc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_rng.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_rtc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_sdmmc.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_spi.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_tim.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_usart.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_usb.c"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Src/stm32f4xx_ll_utils.c"
)

target_include_directories(cube_f4_ll INTERFACE 
    "${CMAKE_SOURCE_DIR}/third_party/cmsis/core/CMSIS/Core/Include"
    "${CMAKE_SOURCE_DIR}/third_party/cmsis/device/st/f4/Include"
    "${CMAKE_SOURCE_DIR}/third_party/hal/st/f4/Inc"
)

target_compile_definitions(cube_f4_ll INTERFACE
    USE_FULL_LL_DRIVER
)

# Create convenience interface library for both the STM32 CubeF4 HAL and LL sources.
add_library(cube_f4 INTERFACE)

target_link_libraries(cube_f4 INTERFACE
    cube_f4_hal
    cube_f4_ll
)
