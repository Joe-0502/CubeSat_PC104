# VST104-Libraries - VST104 project libraries
This repository contains all of the KiCad libraries created and used in the VST104 project. Therefore you can find it included as a submodule in every other VST140 repository.

***

### VST_symbols
During the work on individual schematics, mostly the KiCad vanilla symbols were used. The custom symbols were created only in special cases where KiCad libraries did not provide the required symbol. These few symbols are listed and described in `VST_symbols` folder.

### VST104_footprints.pretty
To ensure the full controll over all of the used footprints we concluded to design all of them from scratch. All of the footprints listed and described in `VST_footprints.pretty` are created acordingly to relevant device datasheed. These footrpints also respects [KiCad courtyard requirements](https://kicad.org/libraries/klc/F5.3/). Relevant 3D models in both `.step` and `.wrl` format are included for every footprints. Up to this point, only the `.step` variants are configured to work. Other common features of som layers (in [mm]) are:
* SilkScreen: `line thickness: 0.16`
* SilkScreen Text: `thickness: 0.16` ; `width: 0.8` ; `height: 0.8`
* Courtyard: `line thickness: 0.05`
* Fabrication: `line thickness: 0.05`
* Fabrication Text: `thickness: 0.1` ; `width: 0.5` ; `height: 0.5`

### VST104_logos.pretty
Various logos used on the SilkScreen layer are listed in this footprint library.

***

The following table lists all of the created footprints:

| Module Name | Module Description | Module Tags |
| :---: | :--- | :--- |
| `300DP3J1BLKM6QE` | <sub>rocker switch, DPDT, 2A@250VAC, non-illuminated</sub> | <sub>rocker, switch, DPDT</sub> |
| `300SP1J1BLKM2QE` | <sub>rocker switch, SPDT, 5A@125VAC, non-illuminated</sub> | <sub>rocker, switch, SPDT</sub> |
| `BGA-24_FAC024_6.0x8.0mm` | <sub>general Ball Grid Array, 24 pin</sub> | <sub>BGA, BGA-24, FAC024</sub> |
| `CES-126-01-x-D` | <sub>header connector, 02x26 pin, 2.54mm, THT</sub> | <sub>header, 2.54</sub> |
| `CLGA-625_29.0x29.0mm` | <sub>general Ceramic Land Grid Array Package, 625 pin</sub> | <sub>CLGA, CLGA-625</sub> |
| `CLP-104-02-x-D` | <sub>header connector, 02x04 pin, 1.27mm, SMD</sub> | <sub>header, 1.27</sub> |
| `conn_FPC_45x0.5_505110-4591` | <sub>connector FPC, 45 contact, 0.5mm, 0.5A@50V</sub> | <sub>FPC, 0.5</sub> |
| `CP_Radial_D6.3mm_P2.50mm_L11.2mm` | <sub>electrolytic capacitor, radial, THT</sub> | <sub>electrolytic, capacitor</sub> |
| `CP_Radial_D8.0mm_P3.50mm_L11.5mm` | <sub>electrolytic capacitor, radial, THT</sub> | <sub>electrolytic, capacitor</sub> |
| `CX90M-16P` | <sub>connector USB type C, 16 contact, 6A@20V, panel mount</sub> | <sub>USB, USB-C</sub> |
| `C_SMD_0402` | <sub>ceramic capacitor, 0402, SMD</sub> | <sub>ceramic, capacitor, 0402</sub> |
| `C_SMD_0603` | <sub>ceramic capacitor, 0603, SMD</sub> | <sub>ceramic, capacitor, 0603</sub> |
| `C_SMD_0805` | <sub>ceramic capacitor, 0805, SMD</sub> | <sub>ceramic, capacitor, 0805</sub> |
| `C_SMD_1210` | <sub>ceramic capacitor, 1210, SMD</sub> | <sub>ceramic, capacitor, 1210</sub> |
| `DFN-8_3.0x2.0mm_EP` | <sub>general Dual Flat NoLead, exposed ground pad, 8 pin</sub> | <sub>DFN, DFN-8, EP</sub> |
| `D_D7343-31_7.3x4.3x2.8mm` | <sub>diode specific, SMD</sub> | <sub>diode</sub> |
| `D_SMA_DO-214AC_4.3x2.6mm` | <sub>diode SMA (DO-214AC), SMD</sub> | <sub>diode, SMA, DO-214AC</sub> |
| `D_SMB-FN_DO-214AA-FN` | <sub>diode SMB (DO-214AA), SMD</sub> | <sub>diode, SMB, DO-214AA</sub> |
| `D_SMD_0402` | <sub>diode, 0402, SMD</sub> | <sub>diode, 0402</sub> |
| `D_SMD_0603_LED` | <sub>LED diode, 0603, SMD</sub> | <sub>diode, LED, 0603</sub> |
| `D_SMD_0805_LED` | <sub>LED diode, 0805, SMD</sub> | <sub>diode, LED, 0805</sub> |
| `D_SOD-323-F_1.25x1.7mm` | <sub>diode fat Small Outline Diode, 323, SMD</sub> | <sub>diode, SOD, SOD-323</sub> |
| `D_SOD-323_1.65x1.28mm` | <sub>diode Small Outline Diode, 323, SMD</sub> | <sub>diode, SOD, SOD-323</sub> |
| `D_SOD-523_1.2x0.8mm` | <sub>diode Small Outline Diode, 523, SMD</sub> | <sub>diode, SOD, SOD-523</sub> |
| `EG4319` | <sub>slide switch, 4P3T, 0.2A@30V, non-illuminated</sub> | <sub>slide, switch, 4P3T</sub> |
| `ESQ-126-39-x-D` | <sub>header connector, 02x26 pin, 2.54mm, THT,  for PC104</sub> | <sub>header, 2.54, PC104</sub> |
| `FUSE_2AG_01110501Z` | <sub>fuse holder, 2AG 5x15mm , THT</sub> | <sub>fuse, 2AG, 5x15</sub> |
| `G-168S-3011` | <sub>slide switch, 3P3T, 0.5A@125VAC, non-illuminated</sub> | <sub>slide, switch, 3P3T</sub> |
| `HSE_2.0x1.6mm` | <sub>specific High Speed Oscillator, 4 pin</sub> | <sub>HSE, oscillator</sub> |
| `HSOP8_3.9x4.9mm_EP` | <sub>general Heat Sink Small Outline Package, exposed pad, 8 pin</sub> | <sub>HSOP, HSOP-8, HSOP8, EP</sub> |
| `KSE_PN6095` | <sub>test plug (female), 4mm banana jack, 15A, 12.7mm</sub> | <sub>test plug, banana, 4mm</sub> |
| `LQFP-144_20x20mm_P0.5mm` | <sub>generalLow Profile Quad Flat Pack, 144 pin</sub> | <sub>LQFP, LQFP-144</sub> |
| `LSE_3.2x1.5mm` | <sub>specific Low Speed Oscillator, 2 pin</sub> | <sub>LSE, oscillator, chrystal</sub> |
| `L_SMD_0402` | <sub>inductor, 0402, SMD</sub> | <sub>inductor, 0402</sub> |
| `L_SMD_1212_3.0x3.0x1.1mm` | <sub>inductor, 1212, SMD</sub> | <sub>inductor, 1212</sub> |
| `L_SMD_8040_8.0x8.0x4.2mm` | <sub>inductor, 8040, SMD</sub> | <sub>inductor, 8040</sub> |
| `mini_QFN-16-1.8x2.6mm` | <sub>general mini Quad Flat No Leads, 16 pin</sub> | <sub>QFN, mQFN, mQFN-16</sub> |
| `MOLEX_501461-0891` | <sub>connector FPC, 8 contact, 0.5mm, 0.5A@50V</sub> | <sub>FPC, 0.5</sub> |
| `MountingHole_1.2mm` | <sub>mounting hole, 1.2mm</sub>  | <sub>mounting, mounting hole</sub> |
| `MountingHole_3.2mm_M3_Pad_Via` | <sub>mounting hole, 3.2mm (M3), via pads</sub>  | <sub>mounting, mounting hole, M3</sub> |
| `OKL-T6-W12` | <sub>specific DCDC convertor, muRata, SMD</sub> | <sub>DCDC, convertor, muRata</sub> |
| `OKR-T6-W12` | <sub>specific DCDC convertor, muRata, THT</sub> | <sub>DCDC, convertor, muRata</sub> |
| `PC104_footprint` | <sub>VST specific footprint, PC104 board profile</sub> | <sub>VST104, PC104, universal</sub> |
| `PC104_SO-24_universal` | <sub>VST specific universal array, SO-24 extension</sub> | <sub>VST104, PC104, universal</sub> |
| `PC104_universal` | <sub>VST specific universal array, board zero</sub> | <sub>VST104, PC104, universal</sub> |
| `PC104_universal_FPGA` | <sub>VST specific universal array, board delta</sub> | <sub>VST104, PC104, universal</sub> |
| `PC104_universal_OBC&FPGA` | <sub>VST specific universal array board sierra</sub> | <sub>VST104, PC104, universal</sub> |
| `PJ-063AH` | <sub>connector power jack, female, 2x6.5mm, 8A@24V</sub> | <sub>power, jack, 2x6.5</sub> |
| `Power-DI-123_2.8x1.8x1mm` | <sub>power diode 123, SMD</sub> | <sub>diode, power, DI, DI-123</sub> |
| `PXC_1935187` | <sub>fixed terminal block, 5mm, 17.5A@250V, 4 pin, THT</sub> | <sub>terminal, block</sub> |
| `QFN-24_4.0x4.0mm_EP` | <sub>general Quad Flat NoLead, exposed ground pad, 24 pin</sub> | <sub>QFN, QFN-24</sub> |
| `R_SMD_0402` | <sub>resistor, 0402, SMD</sub> | <sub>resistor, 0402</sub> |
| `R_SMD_0603` | <sub>resistor, 0603, SMD</sub> | <sub>resistor, 0603</sub> |
| `R_SMD_0805` | <sub>resistor, 0805, SMD</sub> | <sub>resistor, 0805</sub> |
| `R_SMD_1206` | <sub>resistor, 1206, SMD</sub> | <sub>resistor, 1206</sub> |
| `scope_GND` | <sub>VST specific scope, GND, THT</sub> | <sub>VST104, scope, GND</sub> |
| `scope_PAD_1mm` | <sub>VST specific scope, test pad, 1mm, SMD</sub> | <sub>VST104, scope, test pad</sub> |
| `scope_PAD_2mm` | <sub>VST specific scope, test pad, 2mm, SMD</sub> | <sub>VST104, scope, test pad</sub> |
| `slideSW_SSSS820101_2.0x1.5mm` | <sub>slide switch, DPDT, 0.3A@5VDC, non-illuminated</sub> | <sub>slide, switch, DPDT</sub> |
| `slideSW_SSSS820101_2.0x1.5mm_holes` | <sub>slide switch, DPDT, 0.3A@5VDC, non-illuminated, mount hole</sub> | <sub>slide, switch, DPDT</sub> |
| `SOIC-8_208_5.23x5.23mm` | <sub>general Small Outline Integrated Circuit, 8 pin</sub> | <sub>SOIC, SOIC-8</sub> |
| `SOT-23-3_2.9x1.3mm` | <sub>general Small Outline Transistor, 3 pin</sub> | <sub>SOT, SOT-23</sub> |
| `SOT-23-6L_2.9x1.6mm` | <sub>general Small Outline Transistor, 6 pin</sub> | <sub>SOT, SOT-23</sub> |
| `SOT-323-5L_2.0x1.25mm` | <sub>general Small Outline Transistor, 5 pin</sub> | <sub>SOT, SOT-323</sub> |
| `SOT-363_TSSOP-6_2.0x1.25mm` | <sub>general Thin Small Outline Package, 6 pin</sub> | <sub>SOT, SOT-323</sub> |
| `tactileSW_EVPBB_2.6x1.6mm` | <sub>tactile switch, SPST, 0.3A@15VDC, non-illuminated, SMD</sub> | <sub>tactile, switch, SPST</sub> |
| `TO-252AA_6.5x6.1mm` | <sub>general Transistor Outline, 3 pin</sub> | <sub>transistor, TO, TO-252, TO-252AA</sub> |
| `trace_arc_0.173mm` | <sub>VST specific trace, 180 arc, 0.173mm</sub> | <sub>rocker, switch, DPDT</sub> |
| `trace_arc_0.2mm` | <sub>VST specific trace, 180 arc, 0.2mm</sub> | <sub>VST104, trace, arc</sub> |
| `trimer_3296W_10.0x9.5x10.0` | <sub>general trimmer, vertical, 3 pin</sub> | <sub>VST104, trace, arc</sub> |
| `TSW-102-07-x-D` | <sub>header connector, 02x02 pin, 2.54mm, THT</sub> | <sub>header, 2.54</sub> |
| `TSW-102-07-x-S` | <sub>header connector, 01x02 pin, 2.54mm, THT</sub> | <sub>header, 2.54</sub> |
| `TSW-103-07-x-D` | <sub>header connector, 02x03 pin, 2.54mm, THT</sub> | <sub>header, 2.54</sub> |
| `TSW-104-07-x-S` | <sub>header connector, 01x04 pin, 2.54mm, THT</sub> | <sub>header, 2.54</sub> |
| `TSW-105-07-x-S` | <sub>header connector, 01x05 pin, 2.54mm, THT</sub> | <sub>header, 2.54</sub> |
| `UFBGA-132_7.0x7.0mm` | <sub>general Ultra Fine Ball Grid Array, 132 pin</sub> | <sub>UFBGA, UFBGA-132</sub> |
| `VSON-8_3.0x3.0mm_EP` | <sub>general V-Small Outline No Lead, exposed pad, 8 pin</sub> | <sub>VSON, SON, VSON-8</sub> |
| `VSON-8_3.0x3.0mm_EPV` | <sub>general V-Small Outline No Lead, exposed pad & vias, 8 pin</sub> | <sub>VSON, SON, VSON-8</sub> |
| `WQFN-20_4.0x3.0mm_EP` | <sub>general W-Small Outline No Lead, exposed pad, 20 pin</sub> | <sub>WQFN, QFN, WQFN-20</sub> |
| `WQFN-20_4.0x3.0mm_EPV` | <sub>general W-Small Outline No Lead, exposed pad & vias, 20 pin</sub> | <sub>WQFN, QFN, WQFN-20</sub> |
| `WSON-8_2.0x2.0mm_EP` | <sub>general W-Small Outline No Lead, exposed pad, 8 pin</sub> | <sub>WSON, SON, WSON-8</sub> |
| `WSON-8_2.0x2.0mm_EPV` | <sub>general W-Small Outline No Lead, exposed pad & vias, 8 pin</sub> | <sub>WSON, SON, WSON-8</sub> |
| `WSON-8_6.0x8.0mm_EP` | <sub>general W-Small Outline No Lead, exposed pad, 8 pin</sub> | <sub>WSON, SON, WSON-8</sub> |
| `uDFN-14_3.5x1.35mm` | <sub>general µ Dual Flat No Leads, 14 pin</sub> | <sub>DFN, uDFN, uDFN-14</sub> |
