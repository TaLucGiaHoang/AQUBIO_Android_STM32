
	TOPPERS/ASP Kernel（Release 1.9.3）STM32F7
		Toyohashi Open Platform for Embedded Real-Time Systems/
		Advanced Standard Profile Kernel

TOPPERS/ASP STM32F7は以下の４つのボードのGCCの開発環境に対応します．
本パッケージは個別パッケージであるため、TOPPERS/ASPカーネルターゲット
非依存部パッケージasp-1.9.2.tar.gzと組み合わせて使用してください．
(1)STM社 STM32F746 Discoveryボード
   Chip:STM32F746NGH6
(2)STM社 STM32F746 nucleo-144ボード
   Chip:STM32F746ZGT6
(3)STM社 STM32F767 nucleo-144ボード
   Chip:STM32F767ZT6
(2)STM社 STM32F769 Discoveryボード
   Chip:STM32F769NIH6

ASPの実行形態は以下の２つをサポートします．
実行形態は、コンパイル時の変数DBGENVの設定で変更ができます．
Makefileの設定で変更ができます．

(1)RAM実行：ROMモニタ(rommon)で起動したボードに、UARTを用いて
ASPの実行形式(srec)をダウンロードして実行する形態
rommonのFLASH ROM書込みファイルはtools/rommonに置いてあります．
DBGENVが設定されない場合、またはRAMが設定の場合、
この形態のビルドを行います．

(2)ROM実行：FLASH ROMに書き込んで実行する形態
DBGENVにROMが設定の場合、この形態のビルドを行います．


【ディレクトリ構成】

arch/arm_m_gcc/common
	cortex-mのコモン部
arch/arm_m_gcc/stm32f7xx
	stm32f746のchip依存部
arch/gcc
	gccの環境部
target/stm32f7discovery_gcc
	STM32F746 Discoveryボードのターゲット依存部
target/stm32f769discovery_gcc
	STM32F769 Discoveryボードのターゲット依存部
target/stm32f746nucleo144_gcc
	STM32F746 nucleo-144ボードのターゲット依存部
target/stm32f767nucleo144_gcc
	STM32F767 nucleo-144ボードのターゲット依存部
tools/rommon
	ROMモニタとUARTの設定手順とROMモニタ書き込みバイナリファイル
	ROMモニタのソース
	STM32F46-Discovery用のSDCARD-PLAER2,3の開発環境や
	STM32F746 nucleo-144でarduinoを動作させる環境は
	aspにTOPPERS BASE PLATFORMを組み合わせることで構築できます
	これらはTOPPERS教育コンテンツ基礎１，２の
	STM32F4-Discoveryボード編にて配布します．

