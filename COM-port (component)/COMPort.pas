unit COMPort;

interface

uses
 Windows, Messages, SysUtils, Classes;

{$B-,H+,X+}

type
 TBaudRate = (br1200, br2400, br4800, br9600, br19200, br38400, br57600, br115200);

 TByteSize       = (bs5, bs6, bs7, bs8);
 TCOMErrors      = set of (ceFrame, ceRxParity, ceOverrun, ceBreak, ceIO, ceMode, ceRxOver, ceTxFull);
 TCOMEvents      = set of (evRxChar, evTxEmpty, evRing, evCTS, evDSR, evRLSD, evError, evRx80Full);
 TCOMSignals     = set of (csCTS, csDSR, csRing, csRLSD);
 TParity         = (paNone, paOdd, paEven, paMark, paSpace);
 TStopBits       = (sb1, sb1_5, sb2);
 TSyncMethod     = (smThreadSync, smWindowSync, smNone);
 TComSignalEvent = procedure(Sender: TObject; pState: Boolean) of object;
 TComErrorEvent  = procedure(Sender: TObject; pErrors: TCOMErrors) of object;
 TRxCharEvent    = procedure(Sender: TObject; pCount: Integer) of object;

 TOperationKind = (okWrite, okRead);

 PAsync = ^TAsync;

 TAsync = record
 public
  Overlapped: TOverlapped;
  Kind:       TOperationKind;
  Data:       Pointer;
  Size:       Integer;
 end;

 TCOMPort = class;

 TCOMThread = class(TThread)
 private
  FCOMPort:   TCOMPort;
  FEvents:    TCOMEvents;
  FStopEvent: THandle;
 protected
  procedure DoEvents;
  procedure Execute; override;
  procedure SendEvents;
  procedure Stop;
 public
  constructor Create(ACOMPort: TCOMPort);
 end;

 TCOMTimeouts = class(TPersistent)
 private
  FCOMPort:      TCOMPort;
  FReadInterval: Integer;
  FReadTotalM:   Integer;
  FReadTotalC:   Integer;
  FWriteTotalM:  Integer;
  FWriteTotalC:  Integer;
  procedure SetCOMPort(const pCOMPort: TCOMPort);
  procedure SetReadInterval(const pValue: Integer);
  procedure SetReadTotalM(const pValue: Integer);
  procedure SetReadTotalC(const pValue: Integer);
  procedure SetWriteTotalM(const pValue: Integer);
  procedure SetWriteTotalC(const pValue: Integer);
 protected
  procedure AssignTo(pDest: TPersistent); override;
 public
  constructor Create;
  property COMPort: TCOMPort read FCOMPort;
 published
  property ReadInterval:         Integer read FReadInterval write SetReadInterval;
  property ReadTotalMultiplier:  Integer read FReadTotalM write SetReadTotalM;
  property ReadTotalConstant:    Integer read FReadTotalC write SetReadTotalC;
  property WriteTotalMultiplier: Integer read FWriteTotalM write SetWriteTotalM;
  property WriteTotalConstant:   Integer read FWriteTotalC write SetWriteTotalC;
 end;

 TCOMPort = class(TComponent)
 private
  FBaudRate:     TBaudRate;
  FByteSize:     TByteSize;
  FConnected:    Boolean;
  FCTPriority:   TThreadPriority;
  FEvents:       TCOMEvents;
  FEventThread:  TCOMThread;
  FHandle:       THandle;
  FInBufSize:    Integer;
  FOutBufSize:   Integer;
  FParity:       TParity;
  FPort:         string;
  FStopBits:     TStopBits;
  FSyncMethod:   TSyncMethod;
  FTimeouts:     TCOMTimeouts;
  FUpdate:       Boolean;
  FWindow:       THandle;
  FOnCTSChange:  TComSignalEvent;
  FOnDSRChange:  TComSignalEvent;
  FOnError:      TComErrorEvent;
  FOnRing:       TNotifyEvent;
  FOnRLSDChange: TComSignalEvent;
  FOnRx80Full:   TNotifyEvent;
  FOnRxChar:     TRxCharEvent;
  FOnTxEmpty:    TNotifyEvent;
  procedure CallCTSChange;
  procedure CallDSRChange;
  procedure CallError;
  procedure CallRing;
  procedure CallRLSDChange;
  procedure CallRx80Full;
  procedure CallRxChar;
  procedure CallTxEmpty;
  procedure SetBaudRate(const Value: TBaudRate);
  procedure SetByteSize(const Value: TByteSize);
  procedure SetCTPriority(const Value: TThreadPriority);
  procedure SetInBufSize(const Value: Integer);
  procedure SetOutBufSize(const Value: Integer);
  procedure SetParity(const Value: TParity);
  procedure SetPort(const Value: string);
  procedure SetStopBits(const Value: TStopBits);
  procedure SetSyncMethod(const Value: TSyncMethod);
  procedure SetTimeouts(const Value: TCOMTimeouts);
  procedure WindowMethod(var Message: TMessage);
 protected
  procedure ApplyBuffer;
  procedure ApplyDCB;
  procedure ApplyTimeouts;
  procedure CreateHandle;
  procedure DestroyHandle;
  procedure SetupCOMPort;
 public
  constructor Create(pOwner: TComponent); override;
  destructor Destroy; override;
  procedure AbortAllAsync;
  procedure BeginUpdate;
  procedure ClearBuffer(pInput, pOutput: Boolean);
  function Close: Boolean;
  procedure EndUpdate;
  function InBufCount: Integer;
  function IsAsyncCompleted(pAsyncPtr: PAsync): Boolean;
  function Open: Boolean;
  function OutBufCount: Integer;
  function Read(var pBuffer; pCount: Integer): Integer;
  function ReadStr(pCount: Integer): AnsiString;
  procedure SetDTR(pState: Boolean);
  procedure SetRTS(pState: Boolean);
  function Signals: TCOMSignals;
  function WaitForAsync(var pAsyncPtr: PAsync): Integer;
  function Write(const pBuffer; pCount: Integer): Integer;
  function WriteStr(const pStr: AnsiString): Integer;
  property Connected: Boolean read FConnected;
  property CTPriority: TThreadPriority read FCTPriority write SetCTPriority;
 published
  property BaudRate:     TBaudRate read FBaudRate write SetBaudRate;
  property ByteSize:     TByteSize read FByteSize write SetByteSize;
  property InBufSize:    Integer read FInBufSize write SetInBufSize;
  property OutBufSize:   Integer read FOutBufSize write SetOutBufSize;
  property Parity:       TParity read FParity write SetParity;
  property Port:         string read FPort write SetPort;
  property SyncMethod:   TSyncMethod read FSyncMethod write SetSyncMethod;
  property StopBits:     TStopBits read FStopBits write SetStopBits;
  property Timeouts:     TCOMTimeouts read FTimeouts write SetTimeouts;
  property OnCTSChange:  TComSignalEvent read FOnCTSChange write FOnCTSChange;
  property OnDSRChange:  TComSignalEvent read FOnDSRChange write FOnDSRChange;
  property OnError:      TComErrorEvent read FOnError write FOnError;
  property OnRing:       TNotifyEvent read FOnRing write FOnRing;
  property OnRLSDChange: TComSignalEvent read FOnRLSDChange write FOnRLSDChange;
  property OnRx80Full:   TNotifyEvent read FOnRx80Full write FOnRx80Full;
  property OnRxChar:     TRxCharEvent read FOnRxChar write FOnRxChar;
  property OnTxEmpty:    TNotifyEvent read FOnTxEmpty write FOnTxEmpty;
 end;

 ECOMPort = class(Exception);

procedure InitAsync(var pAsyncPtr: PAsync);
procedure DoneAsync(var pAsyncPtr: PAsync);
procedure EnumCOMPorts(pPorts: TStrings);

procedure Register;

implementation

uses
 Forms;

const
 CM_COMPORT = WM_USER + 1;

 STR_PORT_IS_NOT_EXIST              = 'COM-порт не существует!';
 STR_PORT_IS_BUSY_OTHER_PROGRAM     = 'COM-порт занят другой программой!';
 STR_WRITE_PORT_ERROR               = 'Ошибка записи в COM-порт!';
 STR_READ_PORT_ERROR                = 'Ошибка чтения из COM-порт!';
 STR_INVALID_ASYNC_PARAM            = 'Недопустимый параметр Async!';
 STR_ERROR_PURGE_COMM               = 'Ошибка функции PurgeComm!';
 STR_STATUS_ASYNC_IS_NOT_GET        = 'Не удалось получить статус асинхронной операции!';
 STR_ERROR_SET_COMM_STATE           = 'Ошибка функции SetCommState!';
 STR_ERROR_SET_COMM_TIMEOUTS        = 'Ошибка функции SetCommTimeouts!';
 STR_ERROR_SETUP_COMM               = 'Ошибка функции SetupComm!';
 STR_ERROR_CLEAR_COMM_ERROR         = 'Ошибка функции ClearCommError!';
 STR_ERROR_GET_COMM_MODEM_STATUS    = 'Ошибка функции GetCommModemStatus!';
 STR_ERROR_ESCAPE_COMM_FUNCTION     = 'Ошибка функции EscapeCommFunction!';
 STR_MUST_NO_PROPERTY_FOR_OPEN_PORT = 'Нельзя менять свойство, пока порт открыт!';
 STR_ERROR_REGISTRY                 = 'Ошибка обращения к системному реестру!';

function EventsToInt(const Events: TCOMEvents): Integer;
 begin
  Result := 0;

  if evRxChar in Events then
   Result := Result or EV_RXCHAR;

  if evTxEmpty in Events then
   Result := Result or EV_TXEMPTY;

  if evRing in Events then
   Result := Result or EV_RING;

  if evCTS in Events then
   Result := Result or EV_CTS;

  if evDSR in Events then
   Result := Result or EV_DSR;

  if evRLSD in Events then
   Result := Result or EV_RLSD;

  if evError in Events then
   Result := Result or EV_ERR;

  if evRx80Full in Events then
   Result := Result or EV_RX80FULL;
 end;

function IntToEvents(Mask: Integer): TCOMEvents;
 begin
  Result := [];

  if (EV_RXCHAR and Mask) <> 0 then
   Result := Result + [evRxChar];

  if (EV_TXEMPTY and Mask) <> 0 then
   Result := Result + [evTxEmpty];

  if (EV_RING and Mask) <> 0 then
   Result := Result + [evRing];

  if (EV_CTS and Mask) <> 0 then
   Result := Result + [evCTS];

  if (EV_DSR and Mask) <> 0 then
   Result := Result + [evDSR];

  if (EV_RLSD and Mask) <> 0 then
   Result := Result + [evRLSD];

  if (EV_ERR and Mask) <> 0 then
   Result := Result + [evError];

  if (EV_RX80FULL and Mask) <> 0 then
   Result := Result + [evRx80Full];
 end;

{ TComThread }

constructor TCOMThread.Create(ACOMPort: TCOMPort);
 begin
  inherited Create(True);
  FStopEvent := CreateEvent(nil, True, FALSE, nil);
  FCOMPort := ACOMPort;
  Priority := FCOMPort.CTPriority;
  FreeOnTerminate := FALSE;
  SetCommMask(FCOMPort.FHandle, EventsToInt(FCOMPort.FEvents));
  Resume;
 end;

procedure TCOMThread.Execute;
 var
  EventHandles:               array [0 .. 1] of THandle;
  Overlapped:                 TOverlapped;
  Signaled, BytesTrans, Mask: LongWord;

 begin
  FillChar(Overlapped, SizeOf(Overlapped), 0);
  Overlapped.hEvent := CreateEvent(nil, True, True, nil);
  EventHandles[0] := FStopEvent;
  EventHandles[1] := Overlapped.hEvent;

  repeat
   WaitCommEvent(FCOMPort.FHandle, Mask, @Overlapped);

   Signaled := WaitForMultipleObjects(2, @EventHandles, FALSE, INFINITE);

   if (Signaled = WAIT_OBJECT_0 + 1) and GetOverlappedResult(FCOMPort.FHandle, Overlapped, BytesTrans, FALSE) then
    begin
     FEvents := IntToEvents(Mask);

     case FCOMPort.SyncMethod of
      smThreadSync:
       Synchronize(DoEvents);

      smWindowSync:
       SendEvents;

      smNone:
       DoEvents;
     end;
    end;
  until Signaled <> (WAIT_OBJECT_0 + 1);

  SetCommMask(FCOMPort.FHandle, 0);
  PurgeComm(FCOMPort.FHandle, PURGE_TXCLEAR or PURGE_RXCLEAR);
  CloseHandle(Overlapped.hEvent);
  CloseHandle(FStopEvent);
 end;

procedure TCOMThread.Stop;
 begin
  SetEvent(FStopEvent);
  Sleep(0);
 end;

procedure TCOMThread.SendEvents;
 begin
  if evError in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_ERR, 0);

  if evRxChar in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_RXCHAR, 0);

  if evTxEmpty in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_TXEMPTY, 0);

  if evRing in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_RING, 0);

  if evCTS in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_CTS, 0);

  if evDSR in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_DSR, 0);

  if evRing in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_RLSD, 0);

  if evRx80Full in FEvents then
   SendMessage(FCOMPort.FWindow, CM_COMPORT, EV_RX80FULL, 0);
 end;

procedure TCOMThread.DoEvents;
 begin
  if evError in FEvents then
   FCOMPort.CallError;

  if evRxChar in FEvents then
   FCOMPort.CallRxChar;

  if evTxEmpty in FEvents then
   FCOMPort.CallTxEmpty;

  if evRing in FEvents then
   FCOMPort.CallRing;

  if evCTS in FEvents then
   FCOMPort.CallCTSChange;

  if evDSR in FEvents then
   FCOMPort.CallDSRChange;

  if evRLSD in FEvents then
   FCOMPort.CallRLSDChange;

  if evRx80Full in FEvents then
   FCOMPort.CallRx80Full;
 end;

{ TComTimeouts }

constructor TCOMTimeouts.Create;
 begin
  inherited Create;
  FReadInterval := -1;
  FWriteTotalM := 100;
  FWriteTotalC := 1000;
 end;

procedure TCOMTimeouts.AssignTo(pDest: TPersistent);
 begin
  if pDest is TCOMTimeouts then
   begin
    with TCOMTimeouts(pDest) do
     begin
      FReadInterval := Self.ReadInterval;
      FReadTotalM := Self.ReadTotalMultiplier;
      FReadTotalC := Self.ReadTotalConstant;
      FWriteTotalM := Self.WriteTotalMultiplier;
      FWriteTotalC := Self.WriteTotalConstant;
     end;
   end
  else
   inherited AssignTo(pDest);
 end;

procedure TCOMTimeouts.SetCOMPort(const pCOMPort: TCOMPort);
 begin
  FCOMPort := pCOMPort;
 end;

procedure TCOMTimeouts.SetReadInterval(const pValue: Integer);
 begin
  if pValue <> FReadInterval then
   begin
    FReadInterval := pValue;
    FCOMPort.ApplyTimeouts;
   end;
 end;

procedure TCOMTimeouts.SetReadTotalC(const pValue: Integer);
 begin
  if pValue <> FReadTotalC then
   begin
    FReadTotalC := pValue;
    FCOMPort.ApplyTimeouts;
   end;
 end;

procedure TCOMTimeouts.SetReadTotalM(const pValue: Integer);
 begin
  if pValue <> FReadTotalM then
   begin
    FReadTotalM := pValue;
    FCOMPort.ApplyTimeouts;
   end;
 end;

procedure TCOMTimeouts.SetWriteTotalC(const pValue: Integer);
 begin
  if pValue <> FWriteTotalC then
   begin
    FWriteTotalC := pValue;
    FCOMPort.ApplyTimeouts;
   end;
 end;

procedure TCOMTimeouts.SetWriteTotalM(const pValue: Integer);
 begin
  if pValue <> FWriteTotalM then
   begin
    FWriteTotalM := pValue;
    FCOMPort.ApplyTimeouts;
   end;
 end;

{ TCOMPort }

constructor TCOMPort.Create(pOwner: TComponent);
 begin
  inherited Create(pOwner);
  FComponentStyle := FComponentStyle - [csInheritable];
  FBaudRate := br9600;
  FByteSize := bs8;
  FConnected := FALSE;
  FCTPriority := tpNormal;
  FEvents := [evRxChar, evTxEmpty, evRing, evCTS, evDSR, evRLSD, evError, evRx80Full];
  FHandle := INVALID_HANDLE_VALUE;
  FInBufSize := 2048;
  FOutBufSize := 2048;
  FParity := paNone;
  FPort := 'COM1';
  FStopBits := sb1;
  FSyncMethod := smThreadSync;
  FTimeouts := TCOMTimeouts.Create;
  FTimeouts.SetCOMPort(Self);
  FUpdate := True;
 end;

destructor TCOMPort.Destroy;
 begin
  Close;
  FTimeouts.Free;
  inherited Destroy;
 end;

procedure TCOMPort.CreateHandle;
 begin
  FHandle := CreateFile(PChar('\\.\' + FPort), GENERIC_READ or GENERIC_WRITE, 0, nil, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

  if FHandle = INVALID_HANDLE_VALUE then
   begin
    if GetLastError = ERROR_FILE_NOT_FOUND then
     raise ECOMPort.Create(STR_PORT_IS_NOT_EXIST)
    else
     if GetLastError = ERROR_ACCESS_DENIED then
      raise ECOMPort.Create(STR_PORT_IS_BUSY_OTHER_PROGRAM);
   end;
 end;

procedure TCOMPort.DestroyHandle;
 begin
  if FHandle <> INVALID_HANDLE_VALUE then
   CloseHandle(FHandle);
 end;

procedure TCOMPort.WindowMethod(var Message: TMessage);
 begin
  with Message do
   if Msg = CM_COMPORT then
    try
     if InSendMessage then
      ReplyMessage(0);

     if FConnected then
      case wParam of
       EV_CTS:
        CallCTSChange;

       EV_DSR:
        CallDSRChange;

       EV_RING:
        CallRing;

       EV_RLSD:
        CallRLSDChange;

       EV_RX80FULL:
        CallRx80Full;

       EV_RXCHAR:
        CallRxChar;

       EV_ERR:
        CallError;

       EV_TXEMPTY:
        CallTxEmpty;
      end;

    except
     Application.HandleException(Self);
    end
   else
    Result := DefWindowProc(FWindow, Msg, wParam, lParam);
 end;

procedure TCOMPort.BeginUpdate;
 begin
  FUpdate := FALSE;
 end;

procedure TCOMPort.EndUpdate;
 begin
  if not FUpdate then
   FUpdate := True;
  SetupCOMPort;
 end;

function TCOMPort.Open: Boolean;
 begin
  if not FConnected then
   begin
    CreateHandle;
    FConnected := True;

    try
     SetupCOMPort;
    except
     DestroyHandle;
     FConnected := FALSE;
     raise;
    end;

    if FSyncMethod = smWindowSync then
     FWindow := AllocateHWnd(WindowMethod);

    FEventThread := TCOMThread.Create(Self);
   end;

  Result := FConnected;
 end;

function TCOMPort.Close: Boolean;
 begin
  if FConnected then
   begin
    SetDTR(FALSE);
    SetRTS(FALSE);
    AbortAllAsync;
    FEventThread.Stop;

    if FSyncMethod = smWindowSync then
     DeallocateHWnd(FWindow);

    DestroyHandle;

    FConnected := FALSE;
   end;

  Result := not FConnected;
 end;

procedure TCOMPort.ApplyDCB;
 const
  CBaudRate: array [TBaudRate] of Integer = (CBR_1200, CBR_2400, CBR_4800, CBR_9600, CBR_19200, CBR_38400, CBR_57600, CBR_115200);
 var
  DCB: TDCB;

 begin
  if FConnected and FUpdate then
   begin
    FillChar(DCB, SizeOf(TDCB), 0);
    DCB.DCBlength := SizeOf(TDCB);
    DCB.BaudRate := CBaudRate[FBaudRate];
    DCB.ByteSize := Ord(TByteSize(FByteSize)) + 5;
    DCB.Flags := 1 or ($30 and (DTR_CONTROL_ENABLE shl 4)) or ($3000 and (RTS_CONTROL_ENABLE shl 12));

    if FParity <> paNone then
     DCB.Flags := DCB.Flags or 2;

    DCB.Parity := Ord(TParity(FParity));
    DCB.StopBits := Ord(TStopBits(FStopBits));
    DCB.XonChar := #17;
    DCB.XoffChar := #19;

    if not SetCommState(FHandle, DCB) then
     raise ECOMPort.Create(STR_ERROR_SET_COMM_STATE);
   end;
 end;

procedure TCOMPort.ApplyTimeouts;
 var
  Timeouts: TCommTimeouts;

  function MValue(const Value: Integer): LongWord;
   begin
    if Value < 0 then
     Result := MAXDWORD
    else
     Result := Value;
   end;

 begin
  if FConnected and FUpdate then
   begin
    Timeouts.ReadIntervalTimeout := MValue(FTimeouts.ReadInterval);
    Timeouts.ReadTotalTimeoutMultiplier := MValue(FTimeouts.ReadTotalMultiplier);
    Timeouts.ReadTotalTimeoutConstant := MValue(FTimeouts.ReadTotalConstant);
    Timeouts.WriteTotalTimeoutMultiplier := MValue(FTimeouts.WriteTotalMultiplier);
    Timeouts.WriteTotalTimeoutConstant := MValue(FTimeouts.WriteTotalConstant);

    if not SetCommTimeouts(FHandle, Timeouts) then
     raise ECOMPort.Create(STR_ERROR_SET_COMM_TIMEOUTS);
   end;
 end;

procedure TCOMPort.ApplyBuffer;
 begin
  if FConnected and FUpdate then
   if not SetupComm(FHandle, FInBufSize, FOutBufSize) then
    raise ECOMPort.Create(STR_ERROR_SETUP_COMM);
 end;

procedure TCOMPort.SetupCOMPort;
 begin
  ApplyBuffer;
  ApplyDCB;
  ApplyTimeouts;
 end;

function TCOMPort.InBufCount: Integer;
 var
  Errors:  LongWord;
  ComStat: TCOMStat;

 begin
  if not ClearCommError(FHandle, Errors, @ComStat) then
   raise ECOMPort.Create(STR_ERROR_CLEAR_COMM_ERROR);
  Result := ComStat.cbInQue;
 end;

function TCOMPort.OutBufCount: Integer;
 var
  Errors:  LongWord;
  ComStat: TCOMStat;

 begin
  if not ClearCommError(FHandle, Errors, @ComStat) then
   raise ECOMPort.Create(STR_ERROR_CLEAR_COMM_ERROR);
  Result := ComStat.cbOutQue;
 end;

function TCOMPort.Signals: TCOMSignals;
 var
  Status: LongWord;

 begin
  if not GetCommModemStatus(FHandle, Status) then
   raise ECOMPort.Create(STR_ERROR_GET_COMM_MODEM_STATUS);

  Result := [];

  if (MS_CTS_ON and Status) <> 0 then
   Result := Result + [csCTS];

  if (MS_DSR_ON and Status) <> 0 then
   Result := Result + [csDSR];

  if (MS_RING_ON and Status) <> 0 then
   Result := Result + [csRing];

  if (MS_RLSD_ON and Status) <> 0 then
   Result := Result + [csRLSD];
 end;

procedure TCOMPort.SetDTR(pState: Boolean);
 var
  Act: LongWord;

 begin
  if pState then
   Act := Windows.SetDTR
  else
   Act := Windows.CLRDTR;

  if not EscapeCommFunction(FHandle, Act) then
   raise ECOMPort.Create(STR_ERROR_ESCAPE_COMM_FUNCTION);
 end;

procedure TCOMPort.SetRTS(pState: Boolean);
 var
  Act: LongWord;

 begin
  if pState then
   Act := Windows.SetRTS
  else
   Act := Windows.CLRRTS;

  if not EscapeCommFunction(FHandle, Act) then
   raise ECOMPort.Create(STR_ERROR_ESCAPE_COMM_FUNCTION);
 end;

procedure TCOMPort.ClearBuffer(pInput, pOutput: Boolean);
 var
  Flag: LongWord;

 begin
  Flag := 0;

  if pInput then
   Flag := PURGE_RXCLEAR;

  if pOutput then
   Flag := Flag or PURGE_TXCLEAR;

  if not PurgeComm(FHandle, Flag) then
   raise ECOMPort.Create(STR_ERROR_PURGE_COMM);
 end;

procedure PrepareAsync(pKind: TOperationKind; const pBuffer; pCount: Integer; pAsyncPtr: PAsync);
 begin
  with pAsyncPtr^ do
   begin
    Kind := pKind;

    if Data <> nil then
     FreeMem(Data);

    GetMem(Data, pCount);
    Move(pBuffer, Data^, pCount);
    Size := pCount;
   end;
 end;

function TCOMPort.Write(const pBuffer; pCount: Integer): Integer;
 var
  Success:    Boolean;
  BytesTrans: LongWord;
  AsyncPtr:   PAsync;

 begin
  InitAsync(AsyncPtr);

  try
   if AsyncPtr = nil then
    raise ECOMPort.Create(STR_INVALID_ASYNC_PARAM);

   PrepareAsync(okWrite, pBuffer, pCount, AsyncPtr);
   Success := WriteFile(FHandle, pBuffer, pCount, BytesTrans, @AsyncPtr^.Overlapped) or (GetLastError = ERROR_IO_PENDING);

   if not Success then
    raise ECOMPort.Create(STR_WRITE_PORT_ERROR);

   Result := WaitForAsync(AsyncPtr);
  finally
   DoneAsync(AsyncPtr);
  end;
 end;

function TCOMPort.WriteStr(const pStr: AnsiString): Integer;
 begin
  Result := Write(pStr[1], Length(pStr));
 end;

function TCOMPort.Read(var pBuffer; pCount: Integer): Integer;
 var
  Success:    Boolean;
  BytesTrans: LongWord;
  AsyncPtr:   PAsync;

 begin
  InitAsync(AsyncPtr);

  try
   if AsyncPtr = nil then
    raise ECOMPort.Create(STR_INVALID_ASYNC_PARAM);

   AsyncPtr^.Kind := okRead;
   Success := ReadFile(FHandle, pBuffer, pCount, BytesTrans, @AsyncPtr^.Overlapped) or (GetLastError = ERROR_IO_PENDING);

   if not Success then
    raise ECOMPort.Create(STR_READ_PORT_ERROR);

   Result := WaitForAsync(AsyncPtr);
  finally
   DoneAsync(AsyncPtr);
  end;
 end;

function TCOMPort.ReadStr(pCount: Integer): AnsiString;
 var
  buf:    array of Byte;
  i, cnt: Integer;

 begin
  SetLength(buf, pCount);

  cnt := Read(buf[0], pCount);

  for i := 0 to cnt - 1 do
   Result := Result + AnsiChar(buf[i]);
 end;

function ErrorCode(AsyncPtr: PAsync): AnsiString;
 begin
  if AsyncPtr^.Kind = okRead then
   Result := STR_READ_PORT_ERROR
  else
   Result := STR_WRITE_PORT_ERROR;
 end;

function TCOMPort.WaitForAsync(var pAsyncPtr: PAsync): Integer;
 var
  BytesTrans, Signaled: LongWord;
  Success:              Boolean;

 begin
  if pAsyncPtr = nil then
   raise ECOMPort.Create(STR_INVALID_ASYNC_PARAM);

  Signaled := WaitForSingleObject(pAsyncPtr^.Overlapped.hEvent, INFINITE);
  Success := (Signaled = WAIT_OBJECT_0) and (GetOverlappedResult(FHandle, pAsyncPtr^.Overlapped, BytesTrans, FALSE));

  if not Success then
   raise ECOMPort.Create(ErrorCode(pAsyncPtr));

  Result := BytesTrans;
 end;

procedure TCOMPort.AbortAllAsync;
 begin
  if not PurgeComm(FHandle, PURGE_TXABORT or PURGE_RXABORT) then
   raise ECOMPort.Create(STR_ERROR_PURGE_COMM);
 end;

function TCOMPort.IsAsyncCompleted(pAsyncPtr: PAsync): Boolean;
 var
  BytesTrans: LongWord;

 begin
  if pAsyncPtr = nil then
   raise ECOMPort.Create(STR_INVALID_ASYNC_PARAM);

  Result := GetOverlappedResult(FHandle, pAsyncPtr^.Overlapped, BytesTrans, FALSE);

  if not Result then
   if (GetLastError <> ERROR_IO_PENDING) and (GetLastError <> ERROR_IO_INCOMPLETE) then
    raise ECOMPort.Create(STR_STATUS_ASYNC_IS_NOT_GET);
 end;

procedure TCOMPort.CallCTSChange;
 begin
  if Assigned(FOnCTSChange) then
   FOnCTSChange(Self, csCTS in Signals);
 end;

procedure TCOMPort.CallDSRChange;
 begin
  if Assigned(FOnDSRChange) then
   FOnDSRChange(Self, csDSR in Signals);
 end;

procedure TCOMPort.CallRLSDChange;
 begin
  if Assigned(FOnRLSDChange) then
   FOnRLSDChange(Self, csRLSD in Signals);
 end;

procedure TCOMPort.CallError;
 var
  Errs:    TCOMErrors;
  Errors:  LongWord;
  ComStat: TCOMStat;

 begin
  if not ClearCommError(FHandle, Errors, @ComStat) then
   raise ECOMPort.Create(STR_ERROR_CLEAR_COMM_ERROR);

  Errs := [];

  if (CE_FRAME and Errors) <> 0 then
   Errs := Errs + [ceFrame];

  if ((CE_RXPARITY and Errors) <> 0) and (FParity <> paNone) then
   Errs := Errs + [ceRxParity];

  if (CE_OVERRUN and Errors) <> 0 then
   Errs := Errs + [ceOverrun];

  if (CE_RXOVER and Errors) <> 0 then
   Errs := Errs + [ceRxOver];

  if (CE_TXFULL and Errors) <> 0 then
   Errs := Errs + [ceTxFull];

  if (CE_BREAK and Errors) <> 0 then
   Errs := Errs + [ceBreak];

  if (CE_IOE and Errors) <> 0 then
   Errs := Errs + [ceIO];

  if (CE_MODE and Errors) <> 0 then
   Errs := Errs + [ceMode];

  if (Errs <> []) and Assigned(FOnError) then
   FOnError(Self, Errs);
 end;

procedure TCOMPort.CallRing;
 begin
  if Assigned(FOnRing) then
   FOnRing(Self);
 end;

procedure TCOMPort.CallRx80Full;
 begin
  if Assigned(FOnRx80Full) then
   FOnRx80Full(Self);
 end;

procedure TCOMPort.CallRxChar;
 var
  Count: Integer;

 begin
  Count := InBufCount;
  if (Count > 0) and Assigned(FOnRxChar) then
   FOnRxChar(Self, Count);
 end;

procedure TCOMPort.CallTxEmpty;
 begin
  if Assigned(FOnTxEmpty) then
   FOnTxEmpty(Self);
 end;

procedure TCOMPort.SetBaudRate(const Value: TBaudRate);
 begin
  if Value <> FBaudRate then
   begin
    FBaudRate := Value;
    ApplyDCB;
   end;
 end;

procedure TCOMPort.SetByteSize(const Value: TByteSize);
 begin
  if Value <> FByteSize then
   begin
    FByteSize := Value;
    ApplyDCB;
   end;
 end;

procedure TCOMPort.SetParity(const Value: TParity);
 begin
  if Value <> FParity then
   begin
    FParity := Value;
    ApplyDCB;
   end;
 end;

procedure TCOMPort.SetPort(const Value: string);
 begin
  if FConnected then
   raise ECOMPort.Create(STR_MUST_NO_PROPERTY_FOR_OPEN_PORT)
  else
   if Value <> FPort then
    FPort := Value;
 end;

procedure TCOMPort.SetStopBits(const Value: TStopBits);
 begin
  if Value <> FStopBits then
   begin
    FStopBits := Value;
    ApplyDCB;
   end;
 end;

procedure TCOMPort.SetSyncMethod(const Value: TSyncMethod);
 begin
  if Value <> FSyncMethod then
   begin
    if FConnected then
     raise ECOMPort.Create(STR_MUST_NO_PROPERTY_FOR_OPEN_PORT)
    else
     FSyncMethod := Value;
   end;
 end;

procedure TCOMPort.SetCTPriority(const Value: TThreadPriority);
 begin
  if Value <> FCTPriority then
   begin
    if FConnected then
     raise ECOMPort.Create(STR_MUST_NO_PROPERTY_FOR_OPEN_PORT)
    else
     FCTPriority := Value;
   end;
 end;

procedure TCOMPort.SetInBufSize(const Value: Integer);
 begin
  if Value <> FInBufSize then
   begin
    FInBufSize := Value;
    if (FInBufSize mod 2) = 1 then
     Dec(FInBufSize);
    ApplyBuffer;
   end;
 end;

procedure TCOMPort.SetOutBufSize(const Value: Integer);
 begin
  if Value <> FOutBufSize then
   begin
    FOutBufSize := Value;
    if (FOutBufSize mod 2) = 1 then
     Dec(FOutBufSize);
    ApplyBuffer;
   end;
 end;

procedure TCOMPort.SetTimeouts(const Value: TCOMTimeouts);
 begin
  FTimeouts.Assign(Value);
  ApplyTimeouts;
 end;

procedure InitAsync(var pAsyncPtr: PAsync);
 begin
  New(pAsyncPtr);

  with pAsyncPtr^ do
   begin
    FillChar(Overlapped, SizeOf(TOverlapped), 0);
    Overlapped.hEvent := CreateEvent(nil, True, True, nil);
    Data := nil;
    Size := 0;
   end;
 end;

procedure DoneAsync(var pAsyncPtr: PAsync);
 begin
  with pAsyncPtr^ do
   begin
    CloseHandle(Overlapped.hEvent);

    if Data <> nil then
     FreeMem(Data);
   end;

  Dispose(pAsyncPtr);
  pAsyncPtr := nil;
 end;

procedure EnumCOMPorts(pPorts: TStrings);
 var
  KeyHandle:                    HKEY;
  ErrCode, Index:               Integer;
  ValueName, Data:              string;
  ValueLen, DataLen, ValueType: LongWord;
  TmpPorts:                     TStringList;

 begin
  ErrCode := RegOpenKeyEx(HKEY_LOCAL_MACHINE, 'HARDWARE\DEVICEMAP\SERIALCOMM', 0, KEY_READ, KeyHandle);

  if ErrCode <> ERROR_SUCCESS then
   raise ECOMPort.Create(STR_ERROR_REGISTRY);

  TmpPorts := TStringList.Create;

  try
   Index := 0;

   repeat
    ValueLen := 256;
    DataLen := 256;
    SetLength(ValueName, ValueLen);
    SetLength(Data, DataLen);
    ErrCode := RegEnumValue(KeyHandle, Index, PChar(ValueName), Cardinal(ValueLen), nil, @ValueType, PByte(PChar(Data)), @DataLen);

    if ErrCode = ERROR_SUCCESS then
     begin
      SetLength(Data, DataLen);
      TmpPorts.Add(Data);
      Inc(Index);
     end
    else
     if ErrCode <> ERROR_NO_MORE_ITEMS then
      raise ECOMPort.Create(STR_ERROR_REGISTRY);
   until (ErrCode <> ERROR_SUCCESS);

   TmpPorts.Sort;
   pPorts.Assign(TmpPorts);
  finally
   RegCloseKey(KeyHandle);
   TmpPorts.Free;
  end;
 end;

procedure Register;
 begin
  RegisterComponents('Expand', [TCOMPort]);
 end;

end.
