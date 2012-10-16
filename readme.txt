STM32F405用FreeRTOS + USB CDC VirtualCOM動作サンプルです。
コンパイルには別途FreeRTOSとSTM32F4-Discovery_FW(もしくは、ねむいさんの公開されている"TFT/OLED Control Sample with ChaN's FatFs(SDIO&MMC Driver)"(http://cid-36f4d1230f8a673c.office.live.com/browse.aspx/.Public/src/Cortex/ST/STM32F407xGT/FatFs-LCD))が必要です。
makefileにはそれぞれのディレクトリ、ツールチェインを書き直してください。
更に、STM32F4-Discovery_FWの中の以下のディレクトリを、本コードのlib以下の
同名のディレクトリの内容で置換してください。
STM32_USB_Device_Library
STM32_USB_HOST_Library
STM32_USB_OTG_Driver
