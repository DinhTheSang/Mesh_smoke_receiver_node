################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32XG13X/GCC/subdir.mk
-include protocol/bluetooth/bt_mesh/src/subdir.mk
-include platform/middleware/glib/glib/subdir.mk
-include platform/middleware/glib/dmd/display/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/EFR32BG13P/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: BLE_Mesh_Receive_Data-master_2.axf

# Tool invocations
BLE_Mesh_Receive_Data-master_2.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\dthes\SimplicityStudio\v4_workspace\BLE_Mesh_Receive_Data-master_2\efr32bg13p632f512gm48.ld" -Wl,--undefined,bg_application_properties,--undefined,__Vectors,--undefined,__aeabi_uldivmod,--undefined,ceil,--undefined,__nvm3Base -Xlinker -no-enum-size-warning -Xlinker -no-wchar-size-warning -Xlinker --gc-sections -Xlinker -Map="BLE_Mesh_Receive_Data-master_2.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o BLE_Mesh_Receive_Data-master_2.axf -Wl,--start-group "./platform/Device/SiliconLabs/EFR32BG13P/Source/GCC/startup_efr32bg13p.o" "./dcd.o" "./gatt_db.o" "./graphics.o" "./init_app.o" "./init_board_efr32xg1.o" "./init_mcu_efr32xg1.o" "./lcd_driver.o" "./main.o" "./pti.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/display.o" "./hardware/kit/common/drivers/displayls013b7dh03.o" "./hardware/kit/common/drivers/displaypalemlib.o" "./hardware/kit/common/drivers/i2cspm.o" "./hardware/kit/common/drivers/mx25flash_spi.o" "./hardware/kit/common/drivers/retargetio.o" "./hardware/kit/common/drivers/retargetserial.o" "./hardware/kit/common/drivers/si7013.o" "./hardware/kit/common/drivers/tempsens.o" "./hardware/kit/common/drivers/udelay.o" "./platform/Device/SiliconLabs/EFR32BG13P/Source/system_efr32bg13p.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emlib/src/em_assert.o" "./platform/emlib/src/em_burtc.o" "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_se.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "./platform/middleware/glib/dmd/display/dmd_display.o" "./platform/middleware/glib/glib/bmp.o" "./platform/middleware/glib/glib/glib.o" "./platform/middleware/glib/glib/glib_bitmap.o" "./platform/middleware/glib/glib/glib_circle.o" "./platform/middleware/glib/glib/glib_font_narrow_6x8.o" "./platform/middleware/glib/glib/glib_font_normal_8x8.o" "./platform/middleware/glib/glib/glib_font_number_16x20.o" "./platform/middleware/glib/glib/glib_line.o" "./platform/middleware/glib/glib/glib_polygon.o" "./platform/middleware/glib/glib/glib_rectangle.o" "./platform/middleware/glib/glib/glib_string.o" "./protocol/bluetooth/bt_mesh/src/bg_application_properties.o" "./protocol/bluetooth/bt_mesh/src/mesh_lib.o" "./protocol/bluetooth/bt_mesh/src/mesh_serdeser.o" "C:\Users\dthes\SimplicityStudio\v4_workspace\BLE_Mesh_Receive_Data-master_2\protocol\bluetooth\lib\EFR32XG13X\GCC\libpsstore.a" "C:\Users\dthes\SimplicityStudio\v4_workspace\BLE_Mesh_Receive_Data-master_2\protocol\bluetooth\lib\EFR32XG13X\GCC\binapploader.o" "C:\Users\dthes\SimplicityStudio\v4_workspace\BLE_Mesh_Receive_Data-master_2\protocol\bluetooth\lib\EFR32XG13X\GCC\libcoex.a" "C:\Users\dthes\SimplicityStudio\v4_workspace\BLE_Mesh_Receive_Data-master_2\protocol\bluetooth\lib\EFR32XG13X\GCC\libbluetooth_mesh.a" -lm -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: BLE_Mesh_Receive_Data-master_2.hex'
	arm-none-eabi-objcopy -O ihex "BLE_Mesh_Receive_Data-master_2.axf" "BLE_Mesh_Receive_Data-master_2.hex"
	@echo ' '

	@echo 'Building bin file: BLE_Mesh_Receive_Data-master_2.bin'
	arm-none-eabi-objcopy -O binary "BLE_Mesh_Receive_Data-master_2.axf" "BLE_Mesh_Receive_Data-master_2.bin"
	@echo ' '

	@echo 'Building s37 file: BLE_Mesh_Receive_Data-master_2.s37'
	arm-none-eabi-objcopy -O srec "BLE_Mesh_Receive_Data-master_2.axf" "BLE_Mesh_Receive_Data-master_2.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "BLE_Mesh_Receive_Data-master_2.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) BLE_Mesh_Receive_Data-master_2.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
