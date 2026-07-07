// (c) SpeczPromDizajn LLC, 2026
// INN/KPP 6230093924/623001001
// DUNS 362165653
// 6 Yablochkova Proezd, office 718, Ryazan, Russia
// Web: https://spd.net.ru
// e-mail: info@spd.net.ru
// Phone: +7 (4912) 52-47-14

// v. 24.0

unit UStrings;

interface

uses
 Classes, System.SysUtils;

const
{$IFDEF MSWINDOWS}
 SYSTEM_PATH_DELIVER: Char = '\';
{$ELSE}
 SYSTEM_PATH_DELIVER: Char = '/';
{$ENDIF}
function UpCaseRus(pChr: Char): Char; overload;
function UpCaseRus(pStr: string): string; overload;

function MoneyToStr(Money: Double): string;

// Number - число, которое надо склонять
// nominative - именительный падеж строки
// genitive_singular - родительный падеж единственного числа
// genitive_plural - родительный падеж множественного числа
function DeclensionWord(Number: Integer; const nominative, genitive_singular, genitive_plural: string): string;

// Копирует часть строки, начиная с позиции pos и до конца
function CopyToLast(pSrc: string; pPos: Integer): string;

// Возвращает последний символ строки
function GetLastChar(pStr: string): Char;

// Удаляет последний символ строки
procedure DeleteLastChar(var pStr: string);

// Удаляет все символы с начала строки до заданного символа включительно
procedure DeleteToChar(var pStr: string; pChar: Char);

// Удаляет все символы, начиная с заданного, до конца строки
procedure DeleteFromChar(var pStr: string; pChar: Char);

// Удаляет последний символ строки, если он является символом "\"
procedure DeleteLastSlash(var pStr: string);

function StrSaveToFile(pFileName, pStr: string): Boolean;
function StrLoadFromFile(pFileName: string): string;

// Заменяет разделитель целой и дробной части на символ, заданный в системе
function ReplaceDecimalSeparatorSystem(pStr: string): string;

// Заменяет разделитель целой и дробной части на символ пользователя
function ReplaceDecimalSeparatorUser(pStr: string; pSeparator: Char): string;

// Замена двойных пробелов на одиночные
function ReplaceDoubleSpace(pStr: string): string;

// Замена всех вхождений подстроки pFromStr в строке pTxt на строку pToStr
function ReplaceAllSubStr(pTxt: string; pFromStr, pToStr: string): string;

// Добавление пары имя-значение в конструкцию param=value,param=value,...
// !!! В названии параметра запятые удаляются, в значении - заменяются на пробелы
procedure AddParamValue(var pStr: string; pParam, pValue: string);

// Поиск значения в конструкции param=value,param=value,...
function GetValueByParam(pStr, pParam: string): string;

// Добавление строковой пары имя-значение в конструкцию param@value:param@value:...
// Этот формат используется в сервисе kortex.cloud
procedure AddValueToParamKC(var pStr: string; pName, pValue: string); overload;

// Добавление числовой пары имя-значение в конструкцию param@value:param@value:...
// Этот формат используется в сервисе kortex.cloud
procedure AddValueToParamKC(var pStr: string; pName: string; pValue: Int64); overload;

// Добавление вещественной пары имя-значение в конструкцию param@value:param@value:...
// Этот формат используется в сервисе kortex.cloud
procedure AddValueToParamKC(var pStr: string; pName: string; pValue: Double); overload;

// Добавление логической пары имя-значение в конструкцию param@value:param@value:...
// Этот формат используется в сервисе kortex.cloud
procedure AddValueToParamKC(var pStr: string; pName: string; pValue: Boolean); overload;

// Поиск значения в конструкции param@value:param@value:...
// Этот формат используется в сервисе kortex.cloud
function GetValueByParamKC(pStr, pParam: string): string;

// Удаляет заданные символы из строки
procedure DeleteCharsFromStr(var pStr: string; pChars: TSysCharSet);

// Возвращает путь , по которому расположен исполняемый файл программы
function GetExePath: string;

// Преобразовывает фамилию, имя и отчество в формат Фамилия И.О.
function CreateShortFIO(pFam, pName, pOtch: string): string;

// Преобразование строки с любым разделителем (точка или запятая) в вещественное число
function Str2FloatDef(pStr: string; pDefault: Double): Double;

// Декодирование строки формата name1:name:name3:... в массив строк
// pCnt задаёт максимальное кол-во строк
// Если реально их меньше, ставшиеся заменяются пустыми строками
function DecodeNamesStr(pStr: string; pCnt: Integer): TArray<string>;

// Кодирование массива строк в одну строку формата name1:name:name3:...
function EncodeNamesStr(pSL: TArray<string>): string;

// Удаление пробелов из строки
function DeleteSpaceFromStr(pStr: string): string;

function CheckEAN(pEAN: string): Boolean;

// Преобразование двоично-десятичного числа в двоичное
function BCD2BIN(pValue: Byte): Byte;

// Преобразование двоичного числа в двоично-десятичное
function BIN2BCD(pValue: Byte): Byte;

// Замена запрещённых знаков в XML-строке
function DoCorrectXML(pXml: string): string;

// Форматирование даты/времени
function DateTime2Str(pDateTime: TDateTime; pFormat: string): string;

// Создание случайно строки символов
function CreateRandomUID(pLen: Integer): string;

implementation

uses
 System.StrUtils;

const
 NumEd: array [1 .. 19] of string = ('один ', 'два ', 'три ', 'четыре ', 'пять ', 'шесть ', 'семь ', 'восемь ', 'девять ', 'десять ', 'одиннадцать ',
  'двенадцать ', 'тринадцать ', 'четырнадцать ', 'пятнадцать ', 'шестнадцать ', 'семнадцать ', 'восемнадцать ', 'девятнадцать ');
 NumEd1: array [1 .. 2] of string = ('одна ', 'две ');
 NumDec: array [2 .. 9] of string = ('двадцать ', 'тридцать ', 'сорок ', 'пятьдесят ', 'шестьдесят ', 'семьдесят ', 'восемьдесят ', 'девяносто ');
 NumSot: array [1 .. 9] of string = ('сто ', 'двести ', 'триста ', 'четыреста ', 'пятьсот ', 'шестьсот ', 'семьсот ', 'восемьсот ', 'девятьсот ');
 Xlion0: array [1 .. 3] of string = ('тысяч ', 'миллионов ', 'миллиардов ');
 Xlion1: array [1 .. 3] of string = ('тысяча ', 'миллион ', 'миллиард ');
 Xlion2: array [1 .. 3] of string = ('тысячи ', 'миллиона ', 'миллиарда ');

function UpCaseRus(pChr: Char): Char; overload;
 const
  LoRus = 'абвгдеёжзийклмнопрстуфхцчшщъыьэюя';
  UpRus = 'АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ';

 var
  idx: Integer;

 begin
  idx := LoRus.IndexOf(pChr);

  if idx >= 0 then
   Result := UpRus[idx + Low(UpRus)]
  else
   Result := UpCase(pChr);
 end;

function UpCaseRus(pStr: string): string; overload;
 begin
  for var i := 0 to Length(pStr) - 1 do
   pStr[Low(pStr) + i] := UpCaseRus(pStr[Low(pStr) + i]);

  Result := pStr;
 end;

function MoneyToStr(Money: Double): string;
 var
  Money3, MoneyI: Integer;
  MoneyK, Digit:  Integer;
  LastDigit, T:   Integer;
  Sto, Kop:       string;
  strResult:      string;

 begin
  strResult := '';

  MoneyI := Trunc(Money);
  MoneyK := Round((Money - MoneyI) * 100);

  T := 0;

  while MoneyI > 0 do
   begin
    Money3 := MoneyI mod 1000;
    MoneyI := MoneyI div 1000;

    Sto := '';

    if (Money3 mod 100) < 20 then
     begin
      LastDigit := Money3 mod 20;

      if LastDigit > 0 then
       if (T = 1) and (LastDigit in [1 .. 2]) then
        Sto := NumEd1[LastDigit]
       else
        Sto := NumEd[LastDigit];

      Money3 := Money3 div 100;
     end
    else
     begin
      LastDigit := Money3 mod 10;

      if LastDigit > 0 then
       if (T = 1) and (LastDigit in [1 .. 2]) then
        Sto := NumEd1[LastDigit]
       else
        Sto := NumEd[LastDigit];

      Money3 := Money3 div 10;
      Digit := Money3 mod 10;

      if Digit > 0 then
       Sto := NumDec[Digit] + Sto;

      Money3 := Money3 div 10;
     end;

    if Money3 > 0 then
     Sto := NumSot[Money3] + Sto;

    if T > 0 then
     begin
      if LastDigit = 1 then
       Sto := Sto + Xlion1[T]
      else
       if LastDigit in [2 .. 4] then
        Sto := Sto + Xlion2[T]
       else
        Sto := Sto + Xlion0[T];
     end;

    Inc(T);

    strResult := Sto + strResult;
   end;

  Kop := IntToStr(MoneyK mod 10);
  MoneyK := MoneyK div 10;
  Kop := IntToStr(MoneyK) + Kop;

  strResult := strResult + '. ' + Kop + ' .';

  if strResult <> '' then
   strResult[Low(strResult)] := UpCaseRus(strResult[Low(strResult)]);

  Result := strResult;
 end;

function DeclensionWord(Number: Integer; const nominative, genitive_singular, genitive_plural: string): string;
 var
  last_digit, last_two_digits: Integer;

 begin
  last_digit := Number mod 10;
  last_two_digits := Number mod 100;

  if (last_digit = 1) and (last_two_digits <> 11) then
   Result := nominative
  else
   if ((last_digit = 2) and (last_two_digits <> 12)) or ((last_digit = 3) and (last_two_digits <> 13)) or ((last_digit = 4) and (last_two_digits <> 14)) then
    Result := genitive_singular
   else
    Result := genitive_plural;
 end;

function CopyToLast(pSrc: string; pPos: Integer): string;
 var
  startIndex: Integer;

 begin
  startIndex := pPos - Low(pSrc);

  if (startIndex < 0) or (startIndex >= Length(pSrc)) then
   Exit('');

  Result := pSrc.Substring(startIndex);
 end;

function GetLastChar(pStr: string): Char;
 begin
  if Length(pStr) > 0 then
   Result := pStr[High(pStr)]
  else
   Result := #0;
 end;

procedure DeleteLastChar(var pStr: string);
 begin
  if Length(pStr) > 0 then
   pStr := pStr.Remove(Length(pStr) - 1, 1);
 end;

procedure DeleteToChar(var pStr: string; pChar: Char);
 var
  p: Integer;

 begin
  p := pStr.IndexOf(pChar);

  if p >= 0 then
   pStr := pStr.Remove(0, p + 1);
 end;

procedure DeleteFromChar(var pStr: string; pChar: Char);
 var
  p: Integer;

 begin
  p := pStr.IndexOf(pChar);

  if p >= 0 then
   pStr := pStr.Remove(p, Length(pStr) - p);
 end;

procedure DeleteLastSlash(var pStr: string);
 begin
  if pStr = '' then
   Exit;

  if pStr[High(pStr)] = SYSTEM_PATH_DELIVER then
   pStr := pStr.Remove(Length(pStr) - 1, 1);
 end;

function StrSaveToFile(pFileName, pStr: string): Boolean;
 var
  F: TextFile;

 begin
  try
   AssignFile(F, pFileName);
   Rewrite(F);
   WriteLN(F, pStr);
   CloseFile(F);
   Result := TRUE;
  except
   Result := FALSE;
  end;
 end;

function StrLoadFromFile(pFileName: string): string;
 var
  F: TextFile;

 begin
  try
   AssignFile(F, pFileName);
   Reset(F);
   ReadLN(F, Result);
   CloseFile(F);
  except
   Result := '';
  end;
 end;

function ReplaceDecimalSeparatorSystem(pStr: string): string;
 begin
  pStr := ReplaceStr(pStr, '.', FormatSettings.DecimalSeparator);
  Result := ReplaceStr(pStr, ',', FormatSettings.DecimalSeparator);
 end;

function ReplaceDecimalSeparatorUser(pStr: string; pSeparator: Char): string;
 begin
  Result := ReplaceStr(pStr, FormatSettings.DecimalSeparator, pSeparator);
 end;

function ReplaceDoubleSpace(pStr: string): string;
 begin
  while pStr.IndexOf('  ') >= 0 do
   pStr := ReplaceStr(pStr, '  ', ' ');

  Result := pStr;
 end;

function ReplaceAllSubStr(pTxt: string; pFromStr, pToStr: string): string;
 begin
  if pFromStr = '' then
   Exit(pTxt);

  Result := StringReplace(pTxt, pFromStr, pToStr, [rfReplaceAll]);
 end;

procedure AddParamValue(var pStr: string; pParam, pValue: string);
 begin
  if pStr <> '' then
   pStr := pStr + ',';

  pStr := pStr + ReplaceStr(pParam, ',', '') + '=' + ReplaceStr(pValue, ',', ' ');
 end;

function GetValueByParam(pStr, pParam: string): string;
 var
  p, n: Integer;

 begin
  Result := '';

  p := pStr.IndexOf(',' + pParam);

  if p < 0 then
   p := pStr.IndexOf(pParam)
  else
   Inc(p);

  if p >= 0 then
   begin
    n := pStr.IndexOf(',', p);

    if n < 0 then
     n := Length(pStr);

    pStr := pStr.Substring(p, n - p);

    p := pStr.IndexOf('=');

    if (p >= 0) and (p < Length(pStr) - 1) then
     Result := pStr.Substring(p + 1);
   end
 end;

procedure AddValueToParamKC(var pStr: string; pName, pValue: string);
 begin
  if pStr <> '' then
   pStr := pStr + ':';

  pStr := pStr + pName + '@' + pValue;
 end;

procedure AddValueToParamKC(var pStr: string; pName: string; pValue: Int64);
 begin
  AddValueToParamKC(pStr, pName, IntToStr(pValue));
 end;

procedure AddValueToParamKC(var pStr: string; pName: string; pValue: Double);
 begin
  AddValueToParamKC(pStr, pName, FloatToStr(pValue));
 end;

procedure AddValueToParamKC(var pStr: string; pName: string; pValue: Boolean);
 begin
  AddValueToParamKC(pStr, pName, IntToStr(Ord(pValue)));
 end;

function GetValueByParamKC(pStr, pParam: string): string;
 var
  n, m:     Integer;
  startPos: Integer;

 begin
  Result := '';

  startPos := Low(pStr);
  n := pStr.IndexOf(pParam);

  if n < 0 then
   Exit;

  n := n + Length(pParam); // position of '@' in 0-based coordinates

  if (n < 0) or (n >= Length(pStr)) then
   Exit;

  if pStr[n + startPos] <> '@' then
   Exit;

  m := pStr.IndexOf(':', n + 1);

  if m >= 0 then
   Result := pStr.Substring(n + 1, m - n - 1)
  else
   Result := pStr.Substring(n + 1)
 end;

procedure DeleteCharsFromStr(var pStr: string; pChars: TSysCharSet);
 begin
  for var c in pChars do
   pStr := StringReplace(pStr, string(c), '', [rfReplaceAll]);
 end;

function GetExePath: string;
 begin
  Result := ExtractFilePath(ParamStr(0));
  DeleteLastSlash(Result);
 end;

function CreateShortFIO(pFam, pName, pOtch: string): string;
 begin
  Result := pFam;

  if pName <> '' then
   Result := Result + ' ' + pName.Substring(0, 1) + '.';

  if pOtch <> '' then
   Result := Result + pOtch.Substring(0, 1) + '.';
 end;

function Str2FloatDef(pStr: string; pDefault: Double): Double;
 begin
  Result := StrToFloatDef(ReplaceDecimalSeparatorSystem(pStr), pDefault);
 end;

function DecodeNamesStr(pStr: string; pCnt: Integer): TArray<string>;
 var
  sl: TArray<string>;
  i:  Integer;

 begin
  ReplaceAllSubStr(pStr, '#:', #1);
  ReplaceAllSubStr(pStr, '#@', #2);
  ReplaceAllSubStr(pStr, '##', #3);

  sl := pStr.Split([':'], pCnt);

  for i := 0 to Length(sl) - 1 do
   begin
    ReplaceAllSubStr(sl[i], #1, ':');
    ReplaceAllSubStr(sl[i], #2, '@');
    ReplaceAllSubStr(sl[i], #3, '#');
   end;

  for i := 1 to pCnt - Length(sl) do
   begin
    SetLength(sl, Length(sl) + 1);
    sl[Length(sl) - 1] := '';
   end;

  Result := sl;
 end;

function EncodeNamesStr(pSL: TArray<string>): string;
 var
  s: string;
  i: Integer;

 begin
  if Length(pSL) = 0 then
   Exit('');

  for i := 0 to Length(pSL) - 1 do
   begin
    ReplaceAllSubStr(pSL[i], ':', #1);
    ReplaceAllSubStr(pSL[i], '@', #2);
    ReplaceAllSubStr(pSL[i], '#', #3);
   end;

  for i := 0 to Length(pSL) - 1 do
   begin
    ReplaceAllSubStr(pSL[i], #1, '#:');
    ReplaceAllSubStr(pSL[i], #2, '#@');
    ReplaceAllSubStr(pSL[i], #3, '##');
   end;

  s := pSL[0];

  for i := Low(pSL) + 1 to High(pSL) do
   s := s + ':' + pSL[i];

  Result := s;
 end;

function DeleteSpaceFromStr(pStr: string): string;
 begin
  Result := StringReplace(pStr, ' ', '', [rfReplaceAll]);
 end;

function CheckEAN(pEAN: string): Boolean;
 var
  crcE, crcO, crc: Byte;
  p:               Integer;

  function DigitAt(const s: string; pZeroBasedPos: Integer): Integer;
   var
    ch: Char;

   begin
    ch := s[pZeroBasedPos + Low(s)];

    if (ch >= '0') and (ch <= '9') then
     Result := Ord(ch) - Ord('0')
    else
     Result := 0;
   end;

 begin
  Result := FALSE;

  if StrToInt64Def(pEAN, 0) = 0 then
   Exit(FALSE);

  if Length(pEAN) < 8 then
   begin
    while Length(pEAN) < 8 do
     pEAN := '0' + pEAN;
   end
  else
   if Length(pEAN) < 13 then
    begin
     while Length(pEAN) < 13 do
      pEAN := '0' + pEAN;
    end;

  if Length(pEAN) = 8 then
   begin
    crcO := DigitAt(pEAN, 0) + DigitAt(pEAN, 2) + DigitAt(pEAN, 4) + DigitAt(pEAN, 6);
    crcE := DigitAt(pEAN, 1) + DigitAt(pEAN, 3) + DigitAt(pEAN, 5);
    crc := (10 - (crcO * 3 + crcE) mod 10) mod 10;

    p := 7 + Low(pEAN);
    Result := pEAN[p] = Char(Ord('0') + crc);
   end;

  if Length(pEAN) = 13 then
   begin
    crcE := DigitAt(pEAN, 1) + DigitAt(pEAN, 3) + DigitAt(pEAN, 5) + DigitAt(pEAN, 7) + DigitAt(pEAN, 9) + DigitAt(pEAN, 11);
    crcO := DigitAt(pEAN, 0) + DigitAt(pEAN, 2) + DigitAt(pEAN, 4) + DigitAt(pEAN, 6) + DigitAt(pEAN, 8) + DigitAt(pEAN, 10);

    crc := (10 - (crcE * 3 + crcO) mod 10) mod 10;

    p := 12 + Low(pEAN);
    Result := pEAN[p] = Char(Ord('0') + crc);
   end;
 end;

function BCD2BIN(pValue: Byte): Byte;
 begin
  Result := (pValue shr 4) * 10 + (pValue and $0F);
 end;

function BIN2BCD(pValue: Byte): Byte;
 begin
  Result := ((pValue div 10) shl 4) or (pValue mod 10);
 end;

function DoCorrectXML(pXml: string): string;
 begin
  pXml := ReplaceStr(pXml, '"', '&quot;');
  pXml := ReplaceStr(pXml, '<', '&lt;');
  pXml := ReplaceStr(pXml, '>', '&gt;');
  pXml := ReplaceStr(pXml, '&', '&amp;');
  pXml := ReplaceStr(pXml, #10, '&#10;');
  pXml := ReplaceStr(pXml, #13, '&#13;');
  pXml := ReplaceStr(pXml, #1, '');
  pXml := ReplaceStr(pXml, #8, '');
  pXml := ReplaceStr(pXml, #15, '');

  Result := pXml;
 end;

function DateTime2Str(pDateTime: TDateTime; pFormat: string): string;
 begin
  DateTimeToString(Result, pFormat, pDateTime);
 end;

function CreateRandomUID(pLen: Integer): string;
 const
  charSet = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890';

 begin
  Result := '';

  for var i := 1 to pLen do
   Result := Result + charSet[Random(Length(charSet)) + Low(charSet)];
 end;

end.
