param(
    [string]$VoiceName = 'Microsoft Huihui Desktop'
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Speech
$audioRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'audio'))
$config = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'config.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$clips = [ordered]@{}
foreach ($prop in $config.prompts.PSObject.Properties) {
    $clips[$prop.Name + '.wav'] = [string]$prop.Value
}
foreach ($fileName in $clips.Keys) {
    $target = Join-Path $audioRoot $fileName
    if (Test-Path -LiteralPath $target) { throw "Refusing to overwrite: $target" }
}
$speaker = New-Object System.Speech.Synthesis.SpeechSynthesizer
try {
    $speaker.SelectVoice($VoiceName)
    $speaker.Rate = 0
    $speaker.Volume = 75
    $format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
        44100,
        [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
        [System.Speech.AudioFormat.AudioChannel]::Stereo
    )
    [System.IO.Directory]::CreateDirectory($audioRoot) | Out-Null
    foreach ($fileName in $clips.Keys) {
        $target = Join-Path $audioRoot $fileName
        # Reserve the new file exclusively, then write a PCM WAV header followed
        # by System.Speech's raw audio stream in the requested format.
        $stream = [System.IO.File]::Open($target, [System.IO.FileMode]::CreateNew,
                                       [System.IO.FileAccess]::Write)
        $complete = $false
        try {
            $stream.Write((New-Object byte[] 44), 0, 44)
            $speaker.SetOutputToAudioStream($stream, $format)
            $speaker.Speak($clips[$fileName])
            $speaker.SetOutputToNull()
            $dataLength = [int]($stream.Length - 44)
            $stream.Position = 0
            $writer = New-Object System.IO.BinaryWriter($stream)
            $writer.Write([System.Text.Encoding]::ASCII.GetBytes('RIFF'))
            $writer.Write([int](36 + $dataLength))
            $writer.Write([System.Text.Encoding]::ASCII.GetBytes('WAVEfmt '))
            $writer.Write([int]16)
            $writer.Write([int16]1)
            $writer.Write([int16]2)
            $writer.Write([int]44100)
            $writer.Write([int]176400)
            $writer.Write([int16]4)
            $writer.Write([int16]16)
            $writer.Write([System.Text.Encoding]::ASCII.GetBytes('data'))
            $writer.Write([int]$dataLength)
            $writer.Flush()
            $complete = $true
        } finally {
            $stream.Dispose()
            if (-not $complete) { Remove-Item -LiteralPath $target }
        }
        Write-Output "Created $target"
    }
} finally {
    $speaker.Dispose()
}
