# DebugAnalyzer Command Line User Guide

Please assure the DebugAnalyzer you used is after vesion 2.1.7.

## Usage

    DebugAnalyser [-j0] [-fs] [-fu] [-df bool] [-rf bool] [-ln bool] [-sn bool] [-ts bool] [-it bool]

                  [-sf bool] [-ei bool] [-se bool] [-sl bool] [-rt bool] [-lt int] [-so int] [-sp portNumber]

                  [-br baudrate] [-pr parity] [-sb stopbits] [-fc flowcontrol] [-ar path] [-ri path] [-aa path]

                  [-ap path] [-pi path] [-ae path] [-da path] [-du path] [-dv path] [-dp path] [-fp path]

                  [-ce bool] [-ud bool] [-ucfg bool] [-config path]


## Config file(.cfg) parameters:

The composition and parameters of config file which is transfered to command line are alike to default.cfg.
The parameter and value are divided by a comma.

## Priority:

cmd line parameter > custom .cfg > .default.cfg

## Parameters defination

- -j0: Don't Show UI
- -fs[.cfg: DecSerial]: Decode from serial
- -fu[.cfg: DecUsb]: Decode from usb
- -df[.cfg: SaveDecodedLogFile]: Save decoded log file, default true
- -rf[.cfg: SaveRawFile]: Save raw data file, default true
- -ln[.cfg: LineNumber]: Include line number, default true
- -sn[.cfg: SequenceNumber]: Include sequence number, default true
- -ts[.cfg: TimeStamp]: Include time stamp, default true
- -it[.cfg: InternalTime]: Include internal time, default true
- -sf[.cfg: SaveSnoopFile]: Save snoop file, default true
- -ei[.cfg: EllisysInjection]: Use Ellisys Inject, default true
- -se[.cfg: ShowErrors]: Show windows debug information, default false
- -sl[.cfg: ShowLogs]: Show output logs (if show UI,default true; if hide UI, default false)
- -rt[.cfg: RealtimeDecode]: Real-time decode, default true
	 
     If false, just save raw data as .bin file
- -so[.cfg: ShowSpiltOrientation]: Set UI split orientation (if show UI), default 2

	 1: Horizental

	 2: Vertical

- -lt[.cfg: ShowLogType]: Set log type shown on UI (if show UI), default 1

	 1: ARM
     
	 2: DSP

	 3: ARM & DSP

- -sp[.cfg: Port]: Set serial port number, default COM1
- -br[.cfg: Baud]: Set baud rate, default 2000000
- -pr[.cfg: Parity]: Set parity, default none
- -sb[.cfg: Stop]: Set stop bits, default one
- -fc[.cfg: Flc]: Set flow control, default none
- -ar[.cfg: ARMROMTraceFile]: Set ARM ROM trace file
- -ri[.cfg: ARMROMIndexFile]: Set ARM ROM index file
- -aa[.cfg: ARMAppTraceFile]: Set ARM App trace file
- -ap[.cfg: ARMPatchTraceFile]: Set ARM Patch trace file
- -pi[.cfg: ARMPatchIndexFile]: Set ARM Patch index file
- -ae[.cfg: ARMPatchExtTraceFile]: Set ARM Patch_extension trace file
- -da[.cfg: DSPAppTraceFile]: Set DSP App trace file
- -du[.cfg: DSPAudioTraceFile]: Set DSP Audio trace file
- -dv[.cfg: DSPVoiceTraceFile]: Set DSP Voice trace file
- -dp[.cfg: DSPPatchTraceFile]: Set DSP Patch trace file
- -fp[.cfg: OutputPath]: Indicate .cfa, .bin and .log files' common save path
- (All the above paths should be enclosed in double quotes)
- -ce[.cfg: ParseHCICmdEvt]: Parse HCI command and event, default true
- -ud[.cfg: ShowUndecoded]: Show undecoded data (if show UI), default true
- -ucfg: Indicate whether to update the parameters derived from cmd line to default.cfg
- -config: Use config file to set parameters, default the default.cfg file