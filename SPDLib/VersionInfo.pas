// (c) SpeczPromDizajn LLC, 2025
// INN/KPP 6230093924/623001001
// DUNS 362165653
// 6 Yablochkova Proezd, office 718, Ryazan, Russia
// Web: https://spd.net.ru
// e-mail: info@spd.net.ru
// Phone: +7 (4912) 52-47-14

// v. 4.1

unit VersionInfo;

interface

uses
 Windows, SysUtils;

type
 TVersionInfo = class
  Version: record
  public
   Major:   UInt32; // Целая часть версии
   Minor:   UInt32; // Дробная часть версии
   Release: UInt32; // Текущий релиз
   Build:   UInt32; // Номер сборки
   Str:     string; // Текстовая строка версии в формате %Major.%Minor
  end;

  ProductName: string; // Название программы
  CompanyName: string; // Название компании-разработчика

  constructor Create;
  destructor Destroy; override;
 end;

implementation

function GetBlockInfo(pParams: string): string;
 var
  dump:        UInt32;
  size:        Integer;
  buf:         Pointer;
  ParamPtr:    Pointer;
  TransBuffer: Pointer;
  temp:        UInt32;
  sfile, s:    string;

 begin
  sfile := SysUtils.GetModuleName(HInstance);
  size := GetFileVersionInfoSize(PChar(sfile), dump);

  GetMem(buf, size);

  try
   GetFileVersionInfo(PChar(sfile), 0, size, buf);

   VerQueryValue(buf, '\VarFileInfo\Translation', TransBuffer, dump);

   if dump >= 4 then
    begin
     temp := 0;
     Move(TransBuffer^, temp, 4);
     temp := (temp shr 16) or (temp shl 16);
     s := temp.ToHexString(8);
    end;

   VerQueryValue(buf, PChar('\StringFileInfo\' + s + '\' + pParams), ParamPtr, dump);

   if dump > 1 then
    begin
     SetLength(Result, dump - 1);
     StrLCopy(PChar(Result), ParamPtr, dump);
    end
   else
    Result := '???';
  finally
   FreeMem(buf);
  end;
 end;

constructor TVersionInfo.Create;
 var
  s: string;
  n: Integer;

 begin
  inherited Create;

  ProductName := GetBlockInfo('ProductName');
  CompanyName := GetBlockInfo('CompanyName');

  s := GetBlockInfo('FileVersion');

  n := Pos('.', s);
  Version.Major := StrToInt(Copy(s, 1, n - 1));
  Delete(s, 1, n);

  n := Pos('.', s);
  Version.Minor := StrToInt(Copy(s, 1, n - 1));
  Delete(s, 1, n);

  n := Pos('.', s);
  Version.Release := StrToInt(Copy(s, 1, n - 1));
  Delete(s, 1, n);

  Version.Build := StrToInt(s);

  Version.Str := Format('%d.%d', [Version.Major, Version.Minor]);
 end;

destructor TVersionInfo.Destroy;
 begin
  inherited Destroy;
 end;

end.
