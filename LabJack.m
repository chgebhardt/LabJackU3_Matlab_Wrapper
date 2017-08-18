% Wrapper class for using ONE LabJack U3 via the Exodriver under Linux/MacOSX
%
% inspired by iandol, M.A. Hopcroft and the LabJack forum, you guys are great :-)
%
% 2017/03/08 implemented the exodriver functions, checksum generation, no StreamTO function!
% 
% 2017/03/09 added low-level LED function (manual 5.2.5.4)
%            added low-level ConfigU3 function (manual 5.2.2)
%            added low-level ConfigIO function (manual 5.2.3)
%            added low-level ConfigTimerClock function, (manual: 5.2.4)
%            added FPuint8ArrayToFPDouble for calibration data conversion
%            added low-level ReadMem function
%            added get_calibration_constants
%            added low-level AIN function (manual 5.2.5.1)
%            added low-level WaitShort function (manual 5.2.5.2)
%            added low-level WaitLong function (manual 5.2.5.3)
%            added low-level BitStateRead function (manual 5.2.5.5)
%            added low-level BitStateWrite function (manual 5.2.5.6)
%            added low-level BitDirRead function (manual 5.2.5.7)
%            added low-level BitDirWrite function (manual 5.2.5.8)
%            added low-level PortStateRead function (manual 5.2.5.9)
%            added low-level PortStateWrite function (manual 5.2.5.10)
%            added low-level PortDirRead function (manual 5.2.5.11)
%            added low-level PortDirWrite function (manual 5.2.5.12)
%            added low-level DAC#(8-bit) function (manual 5.2.5.13)
%            added low-level DAC#(16-bit) function (manual 5.2.5.14)
%            added low-level Timer function (manual 5.2.5.15)
%            added low-level TimerConfig function (manual 5.2.5.16)
%            added low-level Counter function (manual 5.2.5.17)
%            added low-level Reset function, (manual: 5.2.9)
% 
% 2017/03/10 added error_handling function (called from within each
%            function)
%            added generic feedback function
% 2017/03/15 feedback function done
%
% Note: class currently only opens device if only one device was found and if this device is a U3!!!

classdef LabJack < handle
    
    % properties
    properties(GetAccess = public, SetAccess = public)
        
        connected_devices_total       = [];
        connected_labjack_models      = {};
        datestamp_class_created       = [];
        %dev_descriptor_release_number = [];
        exodriver_version             = [];
        handle                        = [];
        labjack_info                  = struct;
        calibration_constants         = struct;

        byte_sent                     = [];
        byte_received                 = [];
        time_out                      =  0;   % timeout for rawReadTO or rawWriteTO, in milliseconds, 0 = inf
        debug                         = false;
        
        feedback_out_struct           = struct;
        
        isTimer_enabled               = false;
        timerClockBase
        timerClockDivisor
        isCounter_enabled             = false;
        
        isStreaming_enabled           = false;
        
    end
    
    properties (SetAccess = private, GetAccess = public)
        
        exodriver_header       = '/usr/local/include/labjackusb.h'
        exodriver_library      = '/usr/local/lib/liblabjackusb'
        exodriver_functionList = {};
        error_dir              = fileparts(which('LabJack.m'));
        
    end
    
    properties (Constant)
        % max_usb_packet_length = 64; % in bytes
        timeShort =   128e-6;  % 128microsecs for U3C
        timeLong  = 16384e-6;  % 16ms         for U3C
    end
    
    properties (SetAccess = private, GetAccess = private)
        
    end
    
    methods  % public methods
        
        % class Constructor
        function obj = LabJack() % name of constructor function must match name of class
            
            obj.datestamp_class_created = clock();
            
            if ~libisloaded('liblabjackusb')
                loadlibrary(obj.exodriver_library, obj.exodriver_header);
            end
            
            % store the available low-level library functions in a list
            obj.exodriver_functionList = libfunctions('liblabjackusb', '-full');
            
            % return the total number of connected LabJacks and the count for each connected LabJack model
            [obj.connected_devices_total, obj.connected_labjack_models] = GetDevCounts(obj);
            
            % get the Exodriver version
            obj.exodriver_version = GetLibraryVersion(obj);
            
            % opens the LabJack automatically when the class is created
            obj.OpenDevice;
            
        end % end Constructor function
        
        
        % ===================================================================
        %
        % Implement the non-deprecated Exodriver funtions as stored
        % in the header labjackusb.h in alphabetic order
        %
        % ===================================================================
        
        %------------------------------------------------------------------
        %
        % void LJUSB_CloseDevice(HANDLE hDevice);
        %
        % Closes the handle of a LabJack USB device
        %
        %------------------------------------------------------------------
        
        function CloseDevice(obj)
            
            if ~isempty(obj.handle)
                
                calllib('liblabjackusb','LJUSB_CloseDevice', obj.handle);
                obj.handle  = [];
                
            else
                fprintf('No handle to close...');
            end
            
        end % end LabJack CloseDevice function
        
        %------------------------------------------------------------------
        %
        % unsigned int LJUSB_GetDevCount(unsigned long ProductID);
        %
        % Returns the total number of LabJack USB devices connected.
        %
        % ProductID = The product ID of the devices you want to get the count of
        %
        % Note: Implemented but not used in this class
        %
        %------------------------------------------------------------------
        
        function total_count = GetDevCount(~)
            
            LabJackID   =  3; % e.g. 3 = U3, 6 = U6, 9 = U9, 1 = U12 etc.
            
            total_count = calllib('liblabjackusb','LJUSB_GetDevCount', LabJackID);
            
        end % end LabJack GetDevCount function
        
        %------------------------------------------------------------------
        %
        % [uint, uint]  LJUSB_GetDevCounts(UINT *productCounts, UINT * productIds, UINT n);
        % Returns the total count and the count for each connected LabJack model
        %
        % productCounts = Array of size n that holds the count
        % productIds    = Array of size n which holds the product IDs
        % n             = the size of the arrays
        %
        %------------------------------------------------------------------
        
        function [total_count, models] = GetDevCounts(~)
            
            % creates an array in which the number of LabJack devices are
            % stored for each possible LabJack model
            % {'U3','U6','U9','U12','SkyMote Bridge','T7','Digit'};
            
            models      = {'U3'; 'U6'; 'U9'; 'U12'; 'SkyMote Bridge'; 'T7'; 'Digit'};
            model_count = zeros(7,1);
            
            [total_count, model_count] = ...
                calllib('liblabjackusb', 'LJUSB_GetDevCounts', model_count, model_count, 7);
            
            models(:,2) = num2cell(model_count);
            
        end % end LabJack GetDevCounts function
        
        %------------------------------------------------------------------
        %
        % unsigned short LJUSB_GetDeviceDescriptorReleaseNumber(HANDLE hDevice);
        %
        % Returns the device's release number (binary-coded decimal) stored in the
        % device descriptor
        %
        % Note: looks like output will be negative if it failed to get device
        % descriptor release number (see u3.py?), no idea what this is good for!!!
        %
        %------------------------------------------------------------------
        
        function output = GetDeviceDescriptorReleaseNumber(obj)
            
            output = calllib('liblabjackusb', 'LJUSB_GetDeviceDescriptorReleaseNumber', obj.handle);
            
        end % end LabJack GetDeviceDescriptorReleaseNumber function
        
        %------------------------------------------------------------------
        %
        % float LJUSB_GetLibraryVersion(void);
        %
        % Returns the labjackusb library version number
        %
        %------------------------------------------------------------------
        
        function out = GetLibraryVersion(~)
            
            out = calllib('liblabjackusb', 'LJUSB_GetLibraryVersion');
            
        end
        
        %------------------------------------------------------------------
        %
        % bool LJUSB_IsHandleValid(HANDLE hDevice);
        %
        % Returns true if the handle is valid; i.e. it is still connected to a
        % device on the system
        %
        %------------------------------------------------------------------
        
        function validHandle = IsHandleValid(obj)
            
            if ~isempty(obj.handle)
                
                validHandle = calllib('liblabjackusb', 'LJUSB_IsHandleValid', obj.handle);
                
            end
            
        end % end LabJack IsHandleValid function
        
        %--------------------------------------------------------------------------------
        %
        % HANDLE LJUSB_OpenDevice(UINT DevNum, unsigned int dwReserved, unsigned long ProductID);
        %
        % Obtains a handle for a LabJack USB device.  Returns NULL if there is an error.
        % If the device is already open, NULL is returned.
        %
        % DevNum     = The device number of the LabJack USB device you want to open.  For
        %              example, if there is one device connected, set DevNum = 1.  If you
        %              have two devices connected, then set DevNum = 1, or DevNum = 2
        % dwReserved = Not used, set to 0.
        % ProductID  = The product ID of the LabJack USB device. e.g. 3 = U3, 6 = U6 etc.
        %
        % Note: function only opens LabJack if not more than one device was found and if this device is a U3!!!
        %
        %--------------------------------------------------------------------------------
        
        function OpenDevice(obj)
            
            if isempty(obj.handle) && obj.connected_devices_total == 1  && obj.connected_labjack_models{1,2} == 1
                
                device_idx  = 1; % index of the first LabJack U3
                LabJackID   = 3; % e.g. 3 = U3, 6 = U6, 9 = U9, 1 = U12 etc.
                
                obj.handle = calllib('liblabjackusb', 'LJUSB_OpenDevice', device_idx, 0, LabJackID);
                
                if IsHandleValid(obj)
                    fprintf('LabJack opened...\n')
                    
                    % FLAGGED!!!
                    % obj.dev_descriptor_release_number = GetDeviceDescriptorReleaseNumber(obj); % if <0, sth went wrong with getting a device descriptor????
 
                    fprintf('\nLoading Calibration Constants...\n');
                    obj.calibration_constants = get_calibration_constants(obj);
                    
                    fprintf('\nLoading Device Information...\n');
                    obj.labjack_info = ConfigU3_Read(obj);
                    
                    fprintf('\nSet all digital ports to IN by default...')
                    obj.get_feedback(obj.PortDirWrite('0', '0', '0'));

                    fprintf('\nSet all DACs to 0V by default, calling feedback(DAC#16bit)...\n')
                    obj.get_feedback(obj.DAC16bit(0, 0), obj.DAC16bit(1, 0) ); % feedback: Set DAC0 and DAC1 to 0V
                    
                else
                    fprintf('LabJack not opened...\n')
                end
                
            elseif obj.connected_devices_total == 0
                
                fprintf('No LabJack device found! \n');
                
            elseif obj.connected_devices_total >= 1
                
                fprintf('More than one LabJack device found! \n');
                
            elseif obj.connected_labjack_models{1,2} == 0
                
                fprintf('No LabJack U3 found! \n');
                
            elseif obj.connected_labjack_models{1,2} > 1
                
                fprintf('More than one LabJack U3 found! \n');
                
            end
            
        end % end LabJack OpenDevice function
        
        %------------------------------------------------------------------
        %
        % unsigned long LJUSB_ReadTO(HANDLE hDevice, BYTE *pBuff, unsigned long count, unsigned int timeout)
        %
        % Reads from a device with a specified timeout. If the timeout time elapses and
        % no data is transferred the USB request is aborted and the call returns.
        % Returns the number of bytes read, or 0 on error
        %
        % hDevice = The handle for your device
        % pBuff   = The buffer to filled in with bytes from the device.
        % count   = The number of bytes expected to be read.
        % timeout = The USB communication timeout value in milliseconds.
        %           Pass 0 for an unlimited timeout.
        %
        %------------------------------------------------------------------
        
        function byte_received = ReadTO(obj, count)
            
            if IsHandleValid(obj)
                
                byte_received = zeros(1, count);
                
                [out, ~ , byte_received] = calllib('liblabjackusb', 'LJUSB_ReadTO', obj.handle, byte_received, count, obj.time_out);
                if out == 0; fprintf('Reading error!'); end
                
                fprintf('\n Response: %s\n', mat2str(dec2hex(byte_received)) );
                
                % detect bad checksum in response (see manual 5.2.1):
                % i.e. Response: ['B8';'B8';'00';'00';'00';'00';'00';'00';'00';'00']
                
                if all(byte_received(1:2) == hex2dec('B8') )
                    fprintf('\nReadTo: Bad checksum detected!\n');
                end
                
            else
                fprintf('\nReadTo: Invalid device handle!\n');
                return
            end
            
        end
        
        %------------------------------------------------------------------
        %
        % unsigned long LJUSB_StreamTO(HANDLE hDevice, BYTE *pBuff, unsigned long count, unsigned int timeout);
        %
        % Reads from a device's stream interface with a specified timeout.  If the
        % timeout time elapses and no data is transferred the USB request is aborted
        % and the call returns.  Returns the number of bytes read, or 0 on error and
        % errno is set.
        %
        % hDevice = The handle for your device
        % pBuff   = The buffer to be filled in with bytes from the device.
        % count   = The number of bytes expected to be read.
        % timeout = The USB communication timeout value in milliseconds.  Pass 0 for
        %           an unlimited timeout.
        %
        % Note: not finished yet
        %------------------------------------------------------------------
        
        function StreamTO(obj)
            
            if IsHandleValid(obj)
                
                % [streamChars, ~ , obj.streamBuffer] =  calllib('liblabjackusb', 'LJUSB_StreamTO', obj.handle, obj.streamBuffer, obj.streamBufferSize * obj.packagesPerRequest, obj.timeout);
                
            else
                fprintf('StreamTo: invalid device handle');
                return
            end
            
        end
        
        %------------------------------------------------------------------
        %
        % unsigned long LJUSB_WriteTO(HANDLE hDevice, BYTE *pBuff, unsigned long count, unsigned int timeout)
        %
        % Writes to a device with a specified timeout.  If the timeout time elapses and
        % no data is transferred the USB request is aborted and the call returns.
        % Returns the number of bytes written, or 0 on error.
        %
        % hDevice = The handle for your device
        % pBuff   = The buffer to be written to the device.
        % count   = The number of bytes to write.
        % timeout = The USB communication timeout value in milliseconds.
        %           Pass 0 for an unlimited timeout.
        %
        %------------------------------------------------------------------
        
        function byte_sent = WriteTO(obj, byte)
            
            if IsHandleValid(obj)
                
                [in, ~ , byte_sent] = calllib('liblabjackusb', 'LJUSB_WriteTO', obj.handle, byte, length(byte), obj.time_out);
                if in == 0;	fprintf('Writing error!\n');
                    
                else
                    fprintf('\n Sent: %s', mat2str(dec2hex(byte_sent)) );
                end
                
            else
                fprintf('\nWriteTo: invalid device handle');
                return
            end
            
        end
        
        
        % ===================================================================
        %
        % READS the device configuration from ConfigU3, manual: 5.2.2
        %
        % Note: WRITING to flash (affecting the device's power-up values,
        % not the current values) is not implemented
        %
        % ===================================================================
        
        function dev_info = ConfigU3_Read(obj)
            
            % create get info command
            
            cmdGetInfo    = zeros(1, 26);
            cmdGetInfo(2) = hex2dec('F8');  % extended command code 0xF8
            cmdGetInfo(3) = hex2dec('0A');
            cmdGetInfo(4) = hex2dec('08');
            
            command = obj.generate_checksum(cmdGetInfo,'extended');
            
            obj.byte_sent = obj.WriteTO(command);
            
            obj.byte_received = obj.ReadTO(38);
            
            obj.byte_received = double(obj.byte_received);
            
            % Read errorcode byte
            if obj.byte_received(7) > 0
                fprintf('ConfigU3_Read -%s', obj.error_handling(obj.byte_received(7) ) );
                return
            else
                
                fprintf('\nLabjack Configuration Settings:\n');
                
                dev_info.ProductID         = obj.byte_received(20) + obj.byte_received(21)*256;
                fprintf('\n Model:        U%d\n', dev_info.ProductID);
                
                dev_info.LocalID           = obj.byte_received(22);
                fprintf(' Device Index: %d\n', dev_info.LocalID);
                
                dev_info.SerialNumber      = obj.byte_received(16) + obj.byte_received(17)*256 ...
                    + obj.byte_received(18)*65536 + obj.byte_received(19)*16777216;
                fprintf(' SerialNumber: %u\n', dev_info.SerialNumber);
                
                dev_info.FirmwareVersion   = obj.byte_received(10)/100.0 + obj.byte_received(11);
                fprintf('\n FirmwareVersion:   %.3f\n', dev_info.FirmwareVersion);
                
                dev_info.BootloaderVersion = obj.byte_received(12)/100.0 + obj.byte_received(13);
                fprintf(' BootloaderVersion: %.3f\n', dev_info.BootloaderVersion);
                
                dev_info.HardwareVersion   = obj.byte_received(14)/100.0 + obj.byte_received(15);
                fprintf(' HardwareVersion:   %.3f\n', dev_info.HardwareVersion);
                
            end
        end
        
        % ===================================================================
        %
        % ConfigIO, manual: 5.2.3.
        % Writes and reads the current IO configuration.
        % This implementation does not yet assign bits for UART Pins or DAC1Enable !!!
        %
        % FIOAnalog/EIOAnalog: analog input (=1) or digital I/O (=0).
        %
        % e.g. ConfigIO(obj, '0', '0', 0, 1, 0, 4);   configure FIO4 as Counter0
        %
        % ===================================================================
        
        function ConfigIO(obj, FIOAnalog, EIOAnalog, NumberOfTimersEnabled, EnableCounter0, EnableCounter1, TimerCounterPinOffset)
            
            cmdConfigIO      = zeros(1, 12);
            
            cmdConfigIO(2)   = hex2dec('F8');                   % extended command code 0xF8
            cmdConfigIO(3)   = (length(cmdConfigIO) - 6)/2;     % number of data words in package
            cmdConfigIO(4)   = hex2dec('0B');
            
            % command byte 6 generates writemask that determines which of the parameters will be written.
            %
            % e.g. TimerCounterPinOffset = 6; NumberOfTimersEnabled = 1; FIOAnalog = '30', EIOAnalog = '3';
            % DAC1Enable = 0; TimerCounterConfig = 97; EnableCounter1 = 0; EnableCounter0 = 0;
            %
            % Sent:  [0xa8, 0xf8, 0x3, 0xb, 0xa1, 0x0, 0xd, 0x0, 0x61, 0x0, 0x30, 0x3]
            % Response: [0xaa, 0xf8, 0x3, 0xb, 0xa3, 0x0, 0x0, 0x0, 0x61, 0x0, 0x3F, 0x3]
            % b00001101 = 0xd = d13
            
            writeMask = 0;
            
            if hex2dec(EIOAnalog) ~= 0
                writeMask = bitor(writeMask, 1);
                writeMask = bitor(writeMask, 8);
            end
            
            if hex2dec(FIOAnalog) ~= 0
                writeMask  = bitor(writeMask, 1);
                writeMask  = bitor(writeMask, 4);
            end
            
            if TimerCounterPinOffset ~= 0 || EnableCounter1 ~= 0 || EnableCounter0 ~= 0  || NumberOfTimersEnabled ~= 0
                writeMask  = bitor(writeMask, 1);
            end
            
            if hex2dec(EIOAnalog) == 0 && hex2dec(FIOAnalog) == 0
                writeMask  = 13; % b00001101 = 0xd = d13 // writes to FIOAnalog-, EIOAnalog- and TimerCounterConfig-Bit of the mask at once
            end
            
            % dec2bin(writeMask, 8)
            
            cmdConfigIO(7) = writeMask;
            
            % command byte 8 = TimerCounterConfig
            % Bits 0 - 1: Number of timers enabled, Bit 2: EnableCounter0, Bit 3: EnableCounter1,
            % Bits 4 - 7: TimerCounterPinOffset (between 5-8 on U3-HV)
            
            databyte_timer_counter_config  =                                  bitshift(TimerCounterPinOffset, 4);
            databyte_timer_counter_config  = databyte_timer_counter_config  + bitshift(EnableCounter1, 3);
            databyte_timer_counter_config  = databyte_timer_counter_config  + bitshift(EnableCounter0, 2);
            databyte_timer_counter_config  = databyte_timer_counter_config  + bitor(NumberOfTimersEnabled, 0);
            % dec2bin(databyte_timer_counter_config,8)
            
            cmdConfigIO(9)   = databyte_timer_counter_config;
            
            % command byte 11/12:
            % Each bit determines whether that bit of
            % FIO/EIO is analog input (=1) or digital I/O (=0).
            % FIO0-FIO3 are always analog input!!
            % The command byte has to be specified in this
            % implementation as hexadecimal number strings
            % to account for multiple FIOs/EIOs being set to
            % analog/digital, e.g. for FIO4/FIO6 -> b01010000 = 0x50 = d80
            
            cmdConfigIO(11)  = hex2dec(FIOAnalog);
            cmdConfigIO(12)  = hex2dec(EIOAnalog);
            
            command = obj.generate_checksum(cmdConfigIO,'extended');
            
            obj.byte_sent = obj.WriteTO(command);
            
            obj.byte_received = obj.ReadTO(12);
            
            % Return errorcode byte
            if obj.byte_received(7) > 0
                fprintf('ConfigIO -%s \n', obj.error_handling( obj.byte_received(7) ) );
                return
            else
                
                fprintf(' FIOAnalog/EIOAnalog: analog input (=1) or digital I/O (=0)');
                fprintf('\n { NumberOfTimersEnabled = %d, ', NumberOfTimersEnabled);
                fprintf('DAC1Enable = %d, ', obj.byte_received(10));
                fprintf('FIOAnalog = b%s, ', dec2bin(obj.byte_received(11),8));
                fprintf('EIOAnalog = b%s, ',  dec2bin(obj.byte_received(12),8));
                fprintf('TimerCounterConfig = %d,\n', obj.byte_received(9));
                fprintf(' EnableCounter0 = %d, ', EnableCounter0);
                fprintf('EnableCounter1 = %d }\n', EnableCounter1);
                
                if NumberOfTimersEnabled ~= 0
                    obj.isTimer_enabled = true;
                end
                
                if EnableCounter0 || EnableCounter0
                    obj.isCounter_enabled = true;
                end
            end
            
        end % end of low-level ConfigIO function
        
        % ===================================================================
        %
        % ConfigTimerClock, manual: 5.2.4.
        % Writes and reads the timer clock configuration.
        %
        % read_write: reads (0) or writes (1) the clock parameters
        % TimerClockBase:   0:  4MHz // 1: 12MHz// 2: 48MHz(Default)
        %                   3:  1MHz/Divisor // 4:  4MHz/Divisor
        %                   5: 12MHz/Divisor // 6: 48MHz/Divisor
        %                   if TimerClockBase is 3-6, then Counter0 is not available.
        % TimerClockDivisor:  values 0-255
        %                     The base timer clock is divided by this value,
        %                     or divided by 256 if this value is 0.
        %                     Only applies if TimerClockBase is 3-6!
        %
        % Note: TimerClockBase and TimerClockDivisor must be set at the
        % same time!
        %
        % ConfigTimerClock(lj, 1, 6, 183); = 4.002305328 Hz
        % timer_config(lj, 0, 0, 65500); = pulse width = 2.2microsec
        %
        % ===================================================================
        
        function ConfigTimerClock(obj, read_write, ClockBase, ClockDivisor)
            
            U3_TimerClockRate = [4e6 12e6 48e6 1e6 4e6 12e6 48e6]; % TimerClockBase in MHz
            
            if ClockDivisor == 0
                U3_TimerClockDivisor = [1 1 1 256 256 256 256];
            else
                U3_TimerClockDivisor = [1 1 1 ClockDivisor ClockDivisor ClockDivisor ClockDivisor];
            end
            
            cmdConfigTimerClock      = zeros(1, 9);
            
            cmdConfigTimerClock(2)   = hex2dec('F8');         % extended command code 0xF8
            cmdConfigTimerClock(3)   = hex2dec('02');
            cmdConfigTimerClock(4)   = hex2dec('0A');
            
            % command byte 8 = TimerClockConfig
            % Bits 0 - 2: ClockBase
            % Bit      7: determines whether the new ClockBase and
            %             ClockDivisor are written, or if just a read is performed
            
            databyte_TimerClockConfig  = bitshift(read_write, 7);
            databyte_TimerClockConfig  = databyte_TimerClockConfig  + ClockBase;
            % dec2bin(databyte_TimerClockConfig,8)
            
            cmdConfigTimerClock(9)   = databyte_TimerClockConfig;
            cmdConfigTimerClock(10)  = ClockDivisor;
            
            command = obj.generate_checksum(cmdConfigTimerClock,'extended');
            
            obj.byte_sent = obj.WriteTO(command);
            
            obj.byte_received = obj.ReadTO(10);
            
            % Return errorcode byte
            if obj.byte_received(7) > 0
                fprintf('ConfigTimerClock -%s \n', obj.error_handling(obj.byte_received(7) ) );
                return
            else
                obj.timerClockBase    = U3_TimerClockRate(obj.byte_received(9) + 1);
                obj.timerClockDivisor = obj.byte_received(10);
                
                fprintf('TimerClockBase            = %d Hz\n', U3_TimerClockRate(obj.byte_received(9) + 1) );
                fprintf('TimerClockDivisor (0-255) = %d\n', obj.byte_received(10));
                
                fprintf('Resulting ClockRate       = %f Hz\n', (U3_TimerClockRate(obj.byte_received(9) + 1) / U3_TimerClockDivisor(obj.byte_received(9) + 1))/2^16    );
            end
            
        end % end of low-level ConfigTimerClock function
        
        % ===================================================================
        %
        % Feedback low-level subfunctions, manual: 5.2.5.
        %
        % ===================================================================
        
        function data = get_feedback(obj, varargin)
            
            % create commandlist from concatentated name fields in vararg
            
            feedback_commandlist_no       = nargin-1;
            feedback_command_byte_list    = cell(1, feedback_commandlist_no);
            subfunctions_read_byte_length = cell(1, feedback_commandlist_no);
            feedback_called_subfuntions   = cell(1, feedback_commandlist_no);
            
            for ii = 1:feedback_commandlist_no
                
                feedback_called_subfuntions{ii}   = varargin{ii}.func_name;
                feedback_command_byte_list{ii}    = varargin{ii}.cmd;
                subfunctions_read_byte_length{ii} = varargin{ii}.read_length;
                
            end
            
            feedback_command_byte = cell2mat(feedback_command_byte_list); % concatentate cumulative feedback command
            
            obj.byte_sent    = zeros(1, 7); % byte 0-6 are reserved for checksums etc.
            
            obj.byte_sent    = [obj.byte_sent, feedback_command_byte]; % append all the commands received into byte_sent
            
            obj.byte_sent(2) = hex2dec('F8');  % extended command code hex2dec(0xF8) = 248
            
            % number of sent command bytes must be even
            if mod(length(obj.byte_sent), 2)
                obj.byte_sent(end+1) = 0;
            end
            
            % number of command bytes cannot exceed 64 bytes
            if length(obj.byte_sent) > 64
                fprintf('\nERROR: The feedback command you are attempting to send is bigger than 64 bytes ( %s bytes ). Break your commands up into separate calls to get_feedback().', length(obj.byte_sent));
                return
            end
            
            obj.byte_sent(3) = (length(obj.byte_sent))/2 - 3 ; % number of words in package
            
            obj.byte_sent = obj.generate_checksum(obj.byte_sent, 'extended'); % generate checksum for the feedback command

            feedback_read_byte_length = sum(cell2mat(subfunctions_read_byte_length));
            
            % number of read bytes must be even
            if mod(9+feedback_read_byte_length, 2)
                feedback_read_byte_length            = feedback_read_byte_length+1;
                subfunctions_read_byte_length{end+1} = 1;
            end
            
            if 9+feedback_read_byte_length > 64
                
                fprintf('\nERROR: The feedback command you are attempting to send would yield a response that is greater than 64 bytes ( %s bytes ). Break your commands up into separate calls to get_feedback().', 9+feedback_read_byte_length);
                return,
                
            else
                
                obj.WriteTO(obj.byte_sent); % send the feedback command to the LabJack

                obj.byte_received = obj.ReadTO(9+feedback_read_byte_length);
                
            end
            
            % Return errorcode and errorframe bytes
            if obj.byte_received(7) ~= 0
                fprintf('\nFeedback - Error %s in command #%d (%s)\n', obj.error_handling(obj.byte_received(7)), obj.byte_received(8), feedback_called_subfuntions{obj.byte_received(8)});
                return
            end
            
            % splits received byte in parts according to each subfunction given by subfunctions_read_byte_length
            
            tmp = mat2cell(obj.byte_received(10:end), 1, cell2mat(subfunctions_read_byte_length));
            for ii = 1:feedback_commandlist_no
                
                varargin{ii}.out = tmp{ii};
                
            end
            
            obj.feedback_out_struct = varargin;
            
            for ii = 1:feedback_commandlist_no
                
                switch obj.feedback_out_struct{ii}.func_name
                    
                    case 'AIN'
                        
                        if obj.feedback_out_struct{1,ii}.calibration == 1
                            
                            data = double( typecast( uint8( obj.feedback_out_struct{1,ii}.out(1:2) ), 'uint16') );
                            
                            if obj.feedback_out_struct{1,ii}.isLowVoltage == 1
                                if obj.feedback_out_struct{1,ii}.isSingleEnded && ~obj.feedback_out_struct{1,ii}.isSpecialSetting
                                    data = ( data * obj.calibration_constants.LV_AIN(1,1) ) ...
                                        + obj.calibration_constants.LV_AIN(1,2);
                                    
                                elseif obj.feedback_out_struct{1,ii}.isSpecialSetting == 1
                                    % special 0-3.6 volt range for LV-Ports
                                    data = ( data * obj.calibration_constants.LV_AIN(2,1) ) ...
                                        + obj.calibration_constants.LV_AIN(2,2) ...
                                        + obj.calibration_constants.MISC_Temp_Vref(1,2);
                                    
                                else
                                    % differential LV reading
                                    data = ( data * obj.calibration_constants.LV_AIN(2,1) ) ...
                                        + obj.calibration_constants.LV_AIN(2,2);
                                end
                                
                            else
                                % high voltage port
                                if obj.feedback_out_struct{1,ii}.isSingleEnded && ~obj.feedback_out_struct{1,ii}.isSpecialSetting
                                    data = ( data * obj.calibration_constants.HV_AIN(1,1)) ...
                                        + obj.calibration_constants.HV_AIN(1,2);
                                    
                                elseif obj.feedback_out_struct{1,ii}.isSpecialSetting == 1
                                    % "special" -10/20 volt range for HV-Ports
                                    diffR = data * obj.calibration_constants.LV_AIN(2,1) ...
                                        + obj.calibration_constants.LV_AIN(2,2) ...
                                        + obj.calibration_constants.MISC_Temp_Vref(1,2);
                                    data  = diffR * obj.calibration_constants.HV_AIN(1,1) ...
                                        / obj.calibration_constants.LV_AIN(1,1) ...
                                        + obj.calibration_constants.HV_AIN(1,2);
                                    
                                else
                                    % no differential reading possible with HV-ports
                                    fprintf('AIN: Cannot initiate differential readings on high voltage channels!');
                                    return
                                end
                                
                            end
                            
                        else
                            % data un-calibrated
                            data = double(typecast(uint8(obj.obj.feedback_out_struct{1,ii}.out(1:2)),'uint16'));
                            fprintf('AIN: Data are uncalibrated 16bit integers!\n');
                        end
                        
                    case 'BitStateRead'
                        
                        fprintf('\nBitStateRead (IO Number: 0-7 = FIO, 8-15 = EIO, or 16-19 = CIO): \n IONumber: %d\n State (1 = high, 0 = low): %d\n', obj.feedback_out_struct{1,ii}.IONumber, obj.feedback_out_struct{1,ii}.out);
                        
                    case 'BitDirRead'
                        
                        fprintf('\nBitDirRead (IO Number: 0-7 = FIO, 8-15 = EIO, or 16-19 = CIO): \n IONumber: %d\n Direction (1 = digital out, 0 = digital in): %d\n', obj.feedback_out_struct{1,ii}.IONumber, obj.feedback_out_struct{1,ii}.out);
                        
                    case 'PortStateRead'
                        
                        fprintf('\nPortStateRead: \n Response (digital high = 1, digital low = 0 // FIO0-FIO3 are fixed as analog inputs on U3-HV! // just 4 CIOs existent): \n [{ FIO: b%s, EIO b%s, CIO b%s }] \n', ...
                            dec2bin(obj.feedback_out_struct{1,ii}.out(1), 8),...
                            dec2bin(obj.feedback_out_struct{1,ii}.out(2), 8),...
                            dec2bin(obj.feedback_out_struct{1,ii}.out(3), 8) ...
                            );
                        
                    case 'PortDirRead'

                        fprintf('\nPortDirRead (digital out = 1, digital in = 0 // FIO0-FIO3 are fixed as analog inputs on U3-HV! // just 4 CIOs existent): \n [{ FIO: b%s, EIO b%s, CIO b%s }] \n', ...
                            dec2bin(obj.feedback_out_struct{1,ii}.out(1), 8),...
                            dec2bin(obj.feedback_out_struct{1,ii}.out(2), 8),...
                            dec2bin(obj.feedback_out_struct{1,ii}.out(3), 8) ...
                            );

                    case 'Timer'
                        
                        data = ( typecast( uint8( obj.feedback_out_struct{1,ii}.out(1:4) ),'uint32' ) );
                        
                    case 'Counter'
                        
                        data = ( typecast( uint8( obj.feedback_out_struct{1,ii}.out ),'uint32' ) );
                        fprintf('\n%s: %d\n', obj.feedback_out_struct{ii}.Counter_, data);
                        
                end
           end
            
        end
        
        % -------------------------------------------------------------------
        %
        % AIN, manual (IOType = 1), manual: 5.2.5.1
        %
        % reads a single AIN value from the active analog port
        %
        % Note: data calibration not fully tested, also some parameter combinations might not
        % work
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = AIN(obj, pChannel, nChannel, LongSettling, QuickSample, calibration)
            
            if obj.isStreaming_enabled == 0
                
                func = dbstack;
                
                cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
                cmd_struct.func_name = cmd_struct.func_name{2};
                
                cmd_struct.write_length = 3; % length of the read and write bytes see Table 5.2.5-2.
                cmd_struct.read_length  = 2;
                
                isLowVoltage = 0; isSingleEnded = 0; isSpecialSetting = 0;
                
                if (15 >= pChannel) && (pChannel>= 4)
                    isLowVoltage = 1;
                end
                
                if nChannel == 31 || nChannel==199
                    isSingleEnded = 1;
                    
                elseif nChannel == 32
                    nChannel = 30;
                    isSpecialSetting = 1;
                end
                
                cmd_struct.cmd(1) = 1;   % IOType for AIN is 1
                
                % byte 9 = databyte, bits 0-4: pChannel, bit 6: LongSettling, bit 7: QuickSample
                
                databyte  =            bitshift(QuickSample,  7);
                databyte  = databyte + bitshift(LongSettling, 6);
                databyte  = databyte + bitor(pChannel, 0);
                %dec2bin(databyte,8)
                
                cmd_struct.cmd(2) = databyte;
                cmd_struct.cmd(3) = nChannel;
                
                % convert binary data to volts using the calibration
                % constants in obj.calibration_constants depending on the parameters used for reading the input
                
                cmd_struct.calibration      = calibration;
                cmd_struct.isLowVoltage     = isLowVoltage;
                cmd_struct.isSingleEnded    = isSingleEnded;
                cmd_struct.isSpecialSetting = isSpecialSetting;
             
            else
                
                fprintf('AIN: Cannot perform AIN readings while streaming is active!');

            end
            
        end % end of feedback(AIN) subfunction
        
        % -------------------------------------------------------------------
        %
        % WaitShort (IOType = 5), manual: 5.2.5.2
        % LabJack Wait in multiples of 128microsec (U3)
        %
        % time in ms, timeShort = 128microsec for U3
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = WaitShort(obj, time)
            
            if obj.isStreaming_enabled == 0
                
                func = dbstack;
                
                cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
                cmd_struct.func_name = cmd_struct.func_name{2};
                
                cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
                cmd_struct.read_length  = 0;
                
                time = time / 1000; % convert to seconds
                time = ceil(time/obj.timeShort);
                if time > 255
                    time = 255; % truncate to maximum time delay allowed
                end
                
                cmd_struct.cmd(1) = 5; % IOType for WaitShort is 5
                cmd_struct.cmd(2) = time;
             
                fprintf('\nWait for %1.3f sec', time*obj.timeShort);
                
            else
                
                fprintf('WaitShort: Cannot use this function while streaming is active!\n');
                
            end
            
        end % end of feedback(WaitShort) subfunction
        
        % -------------------------------------------------------------------
        %
        % WaitLong (IOType = 6), manual: 5.2.5.3
        % LabJack Wait in multiples of 32ms
        %
        % time in ms, timeLong = 32ms for U3
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = WaitLong(obj, time)
            
            if obj.isStreaming_enabled == 0
                
                func = dbstack;
                
                cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
                cmd_struct.func_name = cmd_struct.func_name{2};
                
                cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
                cmd_struct.read_length  = 0;
                
                time = time / 1000; % convert to seconds
                time = ceil(time/obj.timeLong);
                if time > 255
                    time = 255; % truncate to maximum time delay allowed
                end
                
                cmd_struct.cmd(1) = 6; % IOType for WaitLong is 6
                cmd_struct.cmd(2) = time;
                
                fprintf('\nWait for %1.3f sec', time*obj.timeLong);
                
            else
                
                fprintf('WaitLong: Cannot use this function while streaming is active!\n');
                
            end
            
        end % end of feedback(WaitLong) subfunction
        
        % -------------------------------------------------------------------
        %
        % LED (IOType = 9), manual: 5.2.5.4
        % State: 1=On, 0=Off.
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = LED(obj, state)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;
            
            cmd_struct.cmd(1) = 9;       % IOType for LED is 9
            cmd_struct.cmd(2) = state;
            
            if state == 0
                on_off = 'off';
            elseif state == 1
                on_off = 'on';
            end
            
            fprintf('\nSwitch LED %s', on_off);
            
        end % end of feedback(LED) subfunction
        
        % -------------------------------------------------------------------
        %
        % BitStateRead (IOType = 10), manual: 5.2.5.5
        %
        % This IOType reads the state of a single bit of digital I/O.
        % Only lines configured as digital (not analog) return valid readings.
        % Input:  * IO Number: 0-7 = FIO, 8-15 = EIO, or 16-19 = CIO.
        % Output: * State: 1 = High, 0 = Low.
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = BitStateRead(obj, IONumber)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 1;
            
            cmd_struct.cmd(1)   = 10;        % IOType for BitStateRead is 10
            cmd_struct.cmd(2)   = IONumber;

            cmd_struct.IONumber = IONumber;
            
        end %  end of feedback(BitStateRead) subfunction
        
        % -------------------------------------------------------------------
        %
        % BitStateWrite (IOType = 11), manual: 5.2.5.6
        %
        % This IOType writes the state of a single bit of digital I/O.
        % The direction of the specified line is forced to output.
        %
        % Input:  *IO Number: 0-7 = FIO, 8-15 = EIO, or 16-19 = CIO. *State: 1 = High, 0 = Low.
        % Output: none
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = BitStateWrite(obj, IONumber, state)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;
            
            cmd_struct.cmd(1) = 11;             % IOType for BitStateWrite is 11
            
            % byte 9 = IO_state_byte, bits 0-4: IONumber, bit 7: state
            
            IO_state_byte        =                 bitshift(state, 7);
            IO_state_byte        = IO_state_byte + bitor(IONumber, 0);
            % dec2bin(IO_state_byte,8)
            
            cmd_struct.cmd(2)    = IO_state_byte;
            
        end %  end of feedback(BitStateWrite) subfunction
        
        % -------------------------------------------------------------------
        %
        % BitDirRead (IOType = 12), manual: 5.2.5.7
        %
        % This IOType reads the direction of a single bit of digital I/O.
        % This is the digital direction only, and does not provide any information
        % as to whether the line is configured as digital or analog.
        % Input:  * IO Number: 0-7 = FIO, 8-15 = EIO, or 16-19 = CIO.
        % Output: * Direction: 1 = Output, 0 = Input.
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = BitDirRead(obj, IONumber)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 1;
            
            cmd_struct.cmd(1)   = 12;        % IOType for BitDirRead is 12
            cmd_struct.cmd(2)   = IONumber;

            cmd_struct.IONumber = IONumber;

        end %  end of feedback(BitDirRead) subfunction
        
        % -------------------------------------------------------------------
        %
        % BitDirWrite (IOType = 13), manual: 5.2.5.8
        %
        % This IOType writes the direction of a single bit of digital I/O.
        % Input:  *IO Number: 0-7 = FIO, 8-15 = EIO, or 16-19 = CIO. *Direction: 1 = High, 0 = Low.
        % Output: none
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = BitDirWrite(obj, IONumber, direction)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;

            cmd_struct.cmd(1)       = 13;     % IOType for BitDirWrite is 13
            
            % byte 9 = IO_direction_byte, bits 0-4: IONumber, bit 7: state
            
            IO_direction_byte       =                     bitshift(direction, 7);
            IO_direction_byte       = IO_direction_byte + bitor(IONumber, 0);
            % dec2bin(IO_direction_byte, 8)
            
            cmd_struct.cmd(2)  = IO_direction_byte;
  
        end %  end of feedback(BitDirWrite) subfunction
        
        % -------------------------------------------------------------------
        %
        % PortStateRead (IOType = 26), manual: 5.2.5.9
        %
        % This IOType reads the state of all digital I/O (high = 1, low = 0),
        % where 0-7 = FIO, 8-15 = EIO, and 16-19 = CIO.
        % Only lines configured as digital (not analog) return valid readings.
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = PortStateRead(obj)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 1; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 3;
            
            cmd_struct.cmd(1)   = 26;        % IOType for PortStateRead is 26
            
        end %  end of feedback(PortStateRead) subfunction
        
        % -------------------------------------------------------------------
        %
        % PortStateWrite (IOType = 27), manual: 5.2.5.10
        %
        % This IOType writes the state of all digital I/O (HIGH or LOW),
        % where 0-7=FIO, 8-15=EIO, and 16-19=CIO.
        % 
        % The direction of the selected lines is forced to output!!!
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = PortStateWrite(obj, stateFIO, stateEIO, stateCIO)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 7; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;
            
            cmd_struct.cmd(1)       = 27;       % IOType for PortStateWrite = 27
            
            writeMask               = [255,255,255];
            cmd_struct.cmd(2:4)     = writeMask;
            
            state                   = [bin2dec(stateFIO), bin2dec(stateEIO), bin2dec(stateCIO)];
            cmd_struct.cmd(5:7)     = state;
            
        end %  end of feedback(PortStateWrite) subfunction
        
        % -------------------------------------------------------------------
        %
        % PortDirRead (IOType = 28), manual: 5.2.5.11
        %
        % This IOType reads the direction of all digital I/O (out = 1, in = 0),
        % where 0-7 = FIO, 8-15 = EIO, and 16-19 = CIO.
        % These are the digital directions only, and do not provide
        % any information as to whether the lines are configured as digital or analog.
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = PortDirRead(obj)
            
            func = dbstack;
            
            cmd_struct.func_name    = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name    = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 1; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 3;
            
            cmd_struct.cmd(1)       = 28;        % IOType for PortDirRead is 28
            
        end %  end of feedback(PortDirRead) subfunction
        
        % -------------------------------------------------------------------
        %
        % PortDirWrite (IOType = 29), manual: 5.2.5.12
        %
        % This IOType writes the direction of all digital I/O (1=Output and 0=Input)
        % where 0-7 = FIO, 8-15 = EIO, and 16-19 = CIO.
        % Note that the desired lines must be configured as digital (not analog).
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = PortDirWrite(obj, dirFIO, dirEIO, dirCIO)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 7; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;
            
            cmd_struct.cmd(1)       = 29;    % IOType for PortDirWrite = 29
            
            writeMask               = [255, 255, 255];
            cmd_struct.cmd(2:4)     = writeMask;
            
            direction               = [bin2dec(dirFIO), bin2dec(dirEIO), bin2dec(dirCIO)];
            cmd_struct.cmd(5:7)     = direction;
            
        end %  end of feedback(PortDirWrite) subfunction
        
        % -------------------------------------------------------------------
        %
        % DAC# (8-bit, IOType = 34,35), manual: 5.2.5.13
        %
        % Sets an output voltage for one of the two DAC channels
        % channel: DAC0 or 1
        % voltage: 0-5V, 8bit steps
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = DAC8bit(obj, channel, voltage)
            
            func = dbstack;
            cmd_struct.func_name = strsplit(func(1).name, '.');
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;
            
            % error check inputs
            if channel > 1 || channel < 0
                fprintf('DAC8bit: "channel" must be 0 or 1\n');
                return
            end
            
            if voltage < 0 || voltage > 5
                fprintf('DAC8bit: "voltage" must be 0-5V\n');
                return
            end
            
            % calculate DAC_8bit voltage output from Calibration Data
            
            fprintf('\nDAC8bit: Set DAC%d to %gV', channel, voltage );
            counts = uint8((voltage * obj.calibration_constants.DAC0_1(channel + 1, 1) ) + obj.calibration_constants.DAC0_1(channel + 1, 2));
            fprintf('\nDAC8bit: %gV = %d counts\n', voltage, counts );
            
            if channel == 0
                cmd_struct.cmd(1) = 34; % IOType for  DAC_8bit is 34 or 35
                cmd_struct.DAC_   = 'DAC0';
                
            elseif channel == 1
                cmd_struct.cmd(1) = 35;
                cmd_struct.DAC_   = 'DAC1';
            end
            
            cmd_struct.cmd(2)  = counts;
            
        end %  end of feedback(DAC8bit) subfunction
        
        % -------------------------------------------------------------------
        %
        % DAC# (16-bit, IOType = 38,39), manual: 5.2.5.14
        %
        % Sets an output voltage for one of the two DAC channels
        % channel: DAC0 or 1
        % voltage: 0-5V, 16bit steps
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = DAC16bit(obj, channel, voltage)
            
            func = dbstack;
            cmd_struct.func_name = strsplit(func(1).name, '.');
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 3; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;
            
            % error check inputs
            if channel > 1 || channel < 0
                fprintf('\nDAC16bit: "channel" must be 0 or 1\');
                return
            end
            
            if voltage < 0 || voltage > 5
                fprintf('\nDAC16bit: "voltage" must be 0-5V\n');
                return
            end
            
            % calculate DAC_16bit voltage output from Calibration Data
            
            fprintf('\nDAC16bit: Set DAC%d to %gV', channel, voltage );
            counts = (256 * voltage * obj.calibration_constants.DAC0_1(channel + 1, 1) ) + 256 * obj.calibration_constants.DAC0_1(channel+1, 2);
            fprintf('\nDAC16bit: %gV = %d counts', voltage, uint16(counts) );
            counts = typecast(uint16(counts), 'uint8');
            
            if channel == 0
                cmd_struct.cmd(1) = 38; % IOType for  DAC_16bit is 38 or 39
                cmd_struct.DAC_   = 'DAC0';
                
            elseif channel == 1
                cmd_struct.cmd(1) = 39;
                cmd_struct.DAC_   = 'DAC1';
            end
            
            cmd_struct.cmd(2:3)  = counts;
            
        end %  end of feedback(DAC16bit) subfunction
        
        % -------------------------------------------------------------------
        %
        % Timer#: IOType = 42,44, manual: 5.2.5.15
        %
        % This IOType provides the ability to update/reset a given timer,
        % and read the timer value.
        %
        % Timer_no: Timer0 or Timer1
        % UpdateResetBit: update = 1;
        % value: These values are only updated if the UpdateReset bit is 1.
        % The meaning of this parameter varies with the timer mode.
        % Time: Returns the value from the timer module.
        % This is the value before reset (if reset was done).
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = Timer(obj, timer_no, update_reset_bit, value)
            
            func = dbstack;
            
            cmd_struct.func_name = strsplit(func(1).name, '.'); % first field is always the executed function
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 4; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 1;
            
            if timer_no == 0
                cmd_struct.cmd(1) = 42; % IOType for  DAC_8bit is 34 or 35
                cmd_struct.Timer_ = 'Timer0';
                
            elseif timer_no == 1
                cmd_struct.cmd(1) = 44;
                cmd_struct.Timer_ = 'Timer1';
            end
            
            cmd_struct.cmd(2)     = update_reset_bit;
            
            value                 = typecast(uint16(value),'uint8');
            cmd_struct.cmd(3:4)   = value;
            
        end  %  end of feedback(Timer) subfunction
        
        % -------------------------------------------------------------------
        %
        % Timer#Config: IOType = 43, 45; manual: 5.2.5.16
        %
        % This IOType configures a particular timer.
        %
        % TimerModes:   0   16-bit PWM output
        %               1	 8-bit PWM output
        %               2	Period input (32-bit, rising edges)
        %               3	Period input (32-bit, falling edges)
        %               4	Duty cycle input
        %               5	Firmware counter input
        %               6	Firmware counter input (with debounce)
        %               7	Frequency output
        %               8	Quadrature input
        %               9	Timer stop input (odd timers only)
        %              10	System timer low read (default mode)
        %              11	System timer hight read
        %              12	Period input (16-bit, rising edges)
        %              13	Period input (16-bit, falling edges)
        %
        % value: The meaning of this parameter varies with the timer mode.
        %
        % -------------------------------------------------------------------
        
        function cmd_struct = TimerConfig(obj, timer_no, TimerMode, value)
            
            func = dbstack;
            cmd_struct.func_name = strsplit(func(1).name, '.');
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 4; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 0;
            
            if timer_no == 0
                cmd_struct.cmd(1) = 43; % IOType for Timer#Config is 43 or 45
                cmd_struct.Timer_ = 'Timer0';
                
            elseif timer_no == 1
                cmd_struct.cmd(1) = 45;
                cmd_struct.Timer_ = 'Timer1';
            end
            
            cmd_struct.cmd(2)   = TimerMode;
            cmd_struct.cmd(3:4) = ( typecast( uint16( value ),'uint8' ) ); % LSB & MSB
            
        end %  end of feedback(TimerConfig) subfunction
        
        % -------------------------------------------------------------------
        %
        % Counter#: IOType = 54, 55; manual: 5.2.5.17
        %
        % This IOType reads a hardware counter, and optionally can do a reset.
        %
        % count: Returns the current count from the counter# if enabled.
        % This is the value before reset (if reset was done).
        % counter_no: determines counter# from which to read
        % reset: resets the counter to 0 after reading. (True ( or 1 ) = reset,
        % False ( or 0 ) = Do not reset)
        %
        % -------------------------------------------------------------------
        
        function cmd_struct  = Counter(obj, counter_no, reset)
            
            func = dbstack;
            cmd_struct.func_name = strsplit(func(1).name, '.');
            cmd_struct.func_name = cmd_struct.func_name{2};
            
            cmd_struct.write_length = 2; % length of the read and write bytes see Table 5.2.5-2.
            cmd_struct.read_length  = 4;
            
            if counter_no == 0
                cmd_struct.cmd(1) = 54;                % IOType for Counter is 54 or 55
                cmd_struct.Counter_ = 'Counter0';
                
            elseif counter_no == 1
                cmd_struct.cmd(1) = 55;
                cmd_struct.Counter_ = 'Counter1';
            end
            
            cmd_struct.cmd(2) = reset;
                        
        end %  end of feedback(Counter) subfunction
        
        % ===================================================================
        %
        % ReadMem, manual: 5.2.6.
        %
        % Returns a block from the non-volatile calibration memory (block numbers 0-15)
        % Designed for reading calibration data
        %
        % ===================================================================
        
        function block = ReadMem(obj, BlockNum)
            
            if obj.isStreaming_enabled == 0
                
                cmdReadMem    = zeros(1, 8);
                cmdReadMem(2) = hex2dec('f8');  % extended command code 0xF8
                cmdReadMem(3) = hex2dec('01');
                cmdReadMem(4) = hex2dec('2D');
                cmdReadMem(7) = hex2dec('00');
                cmdReadMem(8) = BlockNum;
                
                command = obj.generate_checksum(cmdReadMem,'extended');
                
                obj.byte_sent = obj.WriteTO(command);
                
                obj.byte_received = obj.ReadTO(40);
                
                % Return errorcode byte
                if obj.byte_received(7) > 0
                    fprintf('ReadMem -%s \n', obj.error_handling(obj.byte_received(7) ) );
                    return
                end
                
                % return received individual data bytes as decimals
                block = uint8( obj.byte_received(9:40) )';
                
            else
                
                fprintf('ReadMem: Cannot use this function while streaming is active\n');
                
            end
            
        end  %  end of ReadMem function
        
        
        % ===================================================================
        %
        % Reset the LabJack, manual: 5.2.9.
        % resetType: soft ('soft') or hard ('hard') reset
        %
        % ===================================================================
        
        function Reset(obj, resetType)
            
            cmdReset            = zeros(1, 4);
            cmdReset(2)         = hex2dec('99'); % reset command code (0x99)
            
            switch resetType
                case {'soft', 'SOFT'}
                    cmdReset(3) = bin2dec('01');
                case {'hard', 'HARD'}
                    cmdReset(3) = bin2dec('10');
            end
            
            command = obj.generate_checksum(cmdReset, 'normal');
            
            obj.byte_sent = obj.WriteTO(command);
            
            obj.byte_received = obj.ReadTO(4);
            
            % Return errorcode byte
            if obj.byte_received(4) > 0
                fprintf('Reset -%s \n', obj.error_handling(obj.byte_received(7) ) );
                return
            elseif obj.byte_received == 0
                fprintf('Reset: %s Labjack reset', resetType);
            end
            
        end %  end of Reset function
        
        % ===================================================================
        %
        % StreamConfig, manual: 5.2.10.
        %
        % Stream mode operates on a table of channels that are scanned at the
        % specified scan rate. Before starting a stream, you need to call this function
        % to configure the table and scan clock
        %
        % numChannels               number of channels sampled per scan (1-25)
        % pChannels/nChannels       For each channel, these two parameters specify the positive and negative voltage measurement point.
        %                           pChannel is 0-7 for FIO0-FIO7, 8-15 for EIO0-EIO15, 30 for temp sensor, 31 for Vreg,
        %                           or 193-224 for digital/timer/counter channels.
        %                           nChannel is 0-7 for FIO0-FIO7, 8-15 for EIO0-EIO15, 30 for Vref, or 31/199 for single-ended.
        %                           should also work with HV-ports (not tested yet, see manual 2.6.1)
        % intern_stream_clock_freq  = [0 1] -> 4 or 48 MHz
        % divideClockBy256          = [0 1] -> false or true
        % resolution                = [0 1 2 3] -> ENOB (effective number of bits) 12.8bit (2500Samples/s) // 11.9bit // 11.3bit // 10.5bit (50kSamples/s)
        % scanFrequency             the frequency in Hz to scan the channel list (pChannels). sampleFrequency (Hz) = scanFrequency * numChannels
        %
        % ===================================================================
        
        % ===================================================================
        %
        % StreamStart, manual: 5.2.11.
        %
        % send the command to initiate data streaming
        %
        % ===================================================================
        
        % ===================================================================
        %
        % StreamData, manual: 5.2.12.
        %
        % Requests streaming data in FIFO-Buffer consisting of several packages
        % at previously specified ScanFrequency
        %
        % Packages Per Request is specified by StreamConfig
        %
        % [ulong, voidPtr, uint8Ptr] LJUSB_Stream(voidPtr, uint8Ptr, ulong)
        %
        % autorecovery-handling, multiple channel or
        % timer/counter screen not checked!!!
        %
        % ===================================================================
        
        % ===================================================================
        %
        % StreamStop, manual: 5.2.13.
        %
        % send the command to stop data streaming
        %
        % ===================================================================
        
        % -------------------------------------------------------------------
        %
        % generate_checksum
        %
        % Calculate checksum for a command sent to the LabJack
        % see the LabJack documentation & u3.c
        % checksums: normal (8bit) and extended (16bit)
        %
        % -------------------------------------------------------------------
        
        function command = generate_checksum(obj, command, type)
            
            switch type
                
                case 'normal'
                     command(1)              = obj.checksum08(command(2:end));
                
                case 'extended'
                    [command(5), command(6)] = obj.checksum16(command(7:end));
                     command(1)              = obj.checksum08(command(2:6  ));
                     
            end
            
        end
        
        % -------------------------------------------------------------------
        %
        % error_handling
        %
        % reads the low-level function errorcodes from a txt-file
        % (see manual table 5.3.1) and throwsback a string describing the error
        %
        % -------------------------------------------------------------------
        
        function str = error_handling(obj, error_byte)
            
            fid = fopen([obj.error_dir,'/LabJack_error_codes.txt']);
            data = textscan(fid,'%s%s%d','delimiter',',');
            
            if fid == -1
                str = [' Reading Error: ', num2str(error_byte), '. Please see the LabJack manual!'];
                
            else
                idx = (data{1,3} == error_byte);
                
                str = [' Reading Error: ', data{1,1}{idx}];
            end
            
            fclose(fid);
            
        end
        
        % -------------------------------------------------------------------
        %
        % get_calibration_constants
        %
        % Get the calibration constants stored on the device
        % Returns structure calibration_constants, with fields: AIN MISC HIRES
        % Each field has a matrix with the relevant values (manual 5.4)
        %
        % -------------------------------------------------------------------
        
        function calibration_constants = get_calibration_constants(obj)
            
            % read U3 Calibration Constants
            calValArray = zeros(4,5); mm = 0;
            
            for kk = 0:4
            
                mm = mm+1;
                block = obj.ReadMem(kk);
                
                for nn = 1:4
                    calValArray(nn, mm) = obj.FPuint8ArrayToFPDouble( block((nn-1)*8+1:nn*8) );
                end
                
            end
            
            % each structure-field will have the slope & offset calibration values for:
            % LV_AIN (SE & Diff) // DAC0_1 // MISC (Temp, Vref, Reserved)  // HV_AIN (AIN0-3)
            % with a bit of reshaping such each row is one porttype,
            % the first column contains the slope (gain, e.g. volts/bit) and the second column
            % contains the offset (e.g. volts) for the calibration
            
            calibration_constants.LV_AIN          = reshape( calValArray(:,   1), 2, 2)';
            calibration_constants.DAC0_1          = reshape( calValArray(:,   2), 2, 2)';
            calibration_constants.MISC_Temp_Vref  = reshape( calValArray(:,   3), 2, 2)';
            calibration_constants.HV_AIN          =          calValArray(:, 4:5);
            
        end
        
        
    end % end of public methods
    
    
    methods ( Static = true ) % do these functions need to be static methods? private?
        
        % -------------------------------------------------------------------
        %
        % checksum08
        % calculate checksum08 for data package see u3.c
        %
        % -------------------------------------------------------------------
        
        function chk = checksum08(in)
            
            % calculate checksum08 for data package
            
            in   = sum(uint16(in));
            quo  = floor(in/2^8);
            remd = rem(in, 2^8);
            in   = quo+remd;
            quo  = floor(in/2^8);
            remd = rem(in, 2^8);
            chk  = quo + remd;
            
        end
        
        % -------------------------------------------------------------------
        %
        % checksum16
        % calculate checksum16 for extended data package see u3.c
        %
        % -------------------------------------------------------------------
        
        function [lsb, msb] = checksum16(in)
            
            % calculate checksum16 needed for extended data package
            
            in  =   sum(uint16(in));
            lsb =   bitand(in, 255);
            msb = bitshift(in,  -8);
            
        end
        
        % -------------------------------------------------------------------
        %
        % Fixed-point conversion
        % convert fixed-point calibration values
        % (see manual chapter 5.4 and FPuint8ArrayToFPDouble in u3.c)
        %
        % -------------------------------------------------------------------
        
        function out = FPuint8ArrayToFPDouble(in)
            
            in = uint32(in);
            
            % assemble the whole number from high order bytes
            value = bitor(in(5), bitshift(in(6),  8));
            value = bitor(value, bitshift(in(7), 16));
            value = bitor(value, bitshift(in(8), 24));
            value = typecast(uint32(value), 'int32'); % to handle negative values
            
            % assemble the fractional value from low order bytes
            fractNum = bitor(in(1),    bitshift(in(2),  8));
            fractNum = bitor(fractNum, bitshift(in(3), 16));
            fractNum = bitor(fractNum, bitshift(in(4), 24));
            
            % assemble the complete value
            out      = double(value) + double(fractNum) / 4294967296;
            
        end
        
    end % end static methods
    
    methods ( Access = private )
        
        % ===================================================================
        %
        % class destructor
        %
        % ===================================================================
        
        function delete(obj)
            
            clc;
            
            % close the LabJack automatically when the class is deleted
            obj.CloseDevice;
            
            fprintf('LabJack closed...\n')
            
        end % end class destructor function
        
    end % end private methods
    
end % end class definition