# dmac
ROS driver for the Evologics Underwater Modems

## Basic commands

 - `ATC` - command mode
 - `ATO` - data mode
 - `AT@ZX` - extended notifications {0:off 1:on}
 - `AT?RP` - promiscuous mode {0:off 1:on}
 - `AT@ZU` - show USBL messages {0:off 1:on}
 - `ATZ` - restart
 - `ATI` - 0:firmware 1:phy_proto,data_prot 7:manufacturer
 - `AT&W` - store settings
 - `AT?L` - source level (from highest to lowest): {0,1,2,3}
 - `AT?C` - waveform (2 for network)
 - `AT?AL` - local address
 - `AT?CA` - sound speed
 - `AT?T` - time of flight (ms)
 - `AT?AM` - maximum number of nodes {2,6,14,30,62,126,254}
 - `ATN` - noise mode
 - `AT?E` - noise profiling

 - Instant messages: 
```
AT*SENDIM,<length>,<destination address>,<flag>,<data>`
DELIVEREDIM,<destination address>`
FAILEDIM,<destination address>`
CANCELLEDIM,<destination address>`
RECVIM,<length>,<source address>,<destination address>,<flag>,
       <duration>,<rssi>,<integrity>,<velocity>,<data>
```
 - Piggyback messages:
```
AT*SENDPBM,<length>,<destination address>,<data>
CANCELLEDPBM,<destination address>
RECVPBM,<length>,<source address>,<destination address>,
        <duration>,<rssi>,<integrity>,<velocity>,<data>
```
 - Position measurements
```
USBLLONG,<current time>,<measurement time>,<remote address>,
<X>,<Y>,<Z>,<E>,<N>,<U>,<roll>,<pitch>,<yaw>,<propagation time>,
<rssi>,<integrity>,<accuracy>
USBLANGLES,<current time>,<measurement time>,<remote_address>,<lbearing>,
<lelevation>,<bearing>,<elevation>,<roll,>,<pitch>,<yaw>,<rssi>,<integrity>,
<accuracy>
```

## Other commands
 - `ATH` - close connection
 - `AT?S` - connection status
 - `AT&V` - device settings
 - `AT?LC` - source level control
 - `AT?G` - gain {0 = normal, 1 = low (-20 dB)}
 - `AT?AR` - remote address
 - `AT?ZC` - cluster size
 - `AT?ZP` - packet time
 - `AT?RC` - retry count
 - `AT?RT` - retry timeout
 - `AT?ZI` - idle timeout
 - `AT?ZS` - stream number
 - `AT?ZSL` - stream number list
 - `AT?ZL` - pool size
 - `AT?ZD` - propagation counter
 - `AT?ZO` - overflow counter
 - `AT?UT` - system time
 - `AT?RI` - IM retry count
 - `AT?DI` - IM delivery status
 - `AT?DA` - wakeup active time
 - `AT?DT` - wakeup period
 - `AT?ZH` - hold timeout
 - `AT?BL` - local to remote bitrate
 - `AT?BR` - remote to local bitrate
 - `AT?E` - rssi (dB respect 1V)
 - `AT?I` - signal integrity
 - `AT?V` - relative velocity
 - `AT?P` - multipath structure
 - `AT&F` - reset factory settings
 - `AT?PID` - Protocol ID
 - `ATN -` noise mode
 - `AT&E` - signal strength measurement
 - `AT?NOISE` - noise analysis `{NOISE,<size>,<sample rate>,<gain>,<rssi>,<data>}`
 - `ATA -` back to listen state
 - `ATS -` switch to deaf state
 - `AT?UP` - estimate pos `<sec>,<addr>,<x>,<y>,<z>`
 - `AT?UPX` - estimate compensated pose `<sec>,<addr>,<E>,<N>,<U>`

## End of line:
 - Ethernet: `\n`
 - Serial: `\r` 
 - Answers: `\r\n` 