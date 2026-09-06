[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Resolve-RequiredTool {
    param(
        [Parameter(Mandatory)]
        [string[]]$Names,

        [Parameter(Mandatory)]
        [string]$InstallHint
    )

    foreach ($name in $Names) {
        $command = Get-Command $name -CommandType Application -ErrorAction SilentlyContinue
        if ($null -ne $command) {
            return $command.Source
        }
    }

    throw "Required tool not found ($($Names -join ', ')). $InstallHint"
}

function Invoke-Checked {
    param(
        [Parameter(Mandatory)]
        [string]$Executable,

        [Parameter(Mandatory)]
        [string[]]$Arguments
    )

    # Windows PowerShell can promote harmless native stderr warnings (notably
    # MiKTeX update notices) to terminating errors when the global preference
    # is Stop. Native tools are authoritative through their process exit code.
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $Executable @Arguments
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }

    if ($exitCode -ne 0) {
        throw "Command failed with exit code $exitCode`: $Executable"
    }
}

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$workingDirectory = Join-Path $repositoryRoot 'local_workspace\pdf-build'
$renderDirectory = Join-Path $workingDirectory 'rendered-pages'
$rawPdf = Join-Path $workingDirectory 'README.raw.pdf'
$candidatePdf = Join-Path $workingDirectory 'README.candidate.pdf'
$publishedPdf = Join-Path $repositoryRoot 'README.pdf'
$defaultsFile = Join-Path $repositoryRoot 'docs\pdf\pandoc.yaml'
$readmeFile = Join-Path $repositoryRoot 'README.md'

$pandoc = Resolve-RequiredTool -Names @('pandoc') -InstallHint 'Install Pandoc from https://pandoc.org/installing.html.'
$xelatex = Resolve-RequiredTool -Names @('xelatex') -InstallHint 'Install XeLaTeX through MiKTeX or TeX Live.'
$ghostscript = Resolve-RequiredTool -Names @('gswin64c', 'gswin32c', 'mgs', 'gs') -InstallHint 'Install Ghostscript, or use a MiKTeX installation that provides mgs.'
$pdfinfo = Resolve-RequiredTool -Names @('pdfinfo') -InstallHint 'Install Poppler, or use the pdfinfo supplied by MiKTeX.'
$pdftoppm = Resolve-RequiredTool -Names @('pdftoppm') -InstallHint 'Install Poppler, or use the pdftoppm supplied by MiKTeX.'

# Pandoc locates the selected PDF engine through PATH. Confirm the resolved
# XeLaTeX directory is available even when the script is started from a shell
# with a minimal PATH.
$xelatexDirectory = Split-Path $xelatex -Parent
if (($env:PATH -split [IO.Path]::PathSeparator) -notcontains $xelatexDirectory) {
    $env:PATH = $xelatexDirectory + [IO.Path]::PathSeparator + $env:PATH
}

New-Item -ItemType Directory -Path $workingDirectory -Force | Out-Null
if (Test-Path -LiteralPath $renderDirectory) {
    Remove-Item -LiteralPath $renderDirectory -Recurse -Force
}
New-Item -ItemType Directory -Path $renderDirectory -Force | Out-Null

Push-Location $repositoryRoot
try {
    Invoke-Checked -Executable $pandoc -Arguments @(
        $readmeFile,
        '--defaults', $defaultsFile,
        '--output', $rawPdf
    )

    $ghostscriptArguments = @(
        '-dBATCH',
        '-dNOPAUSE',
        '-dSAFER',
        '-sDEVICE=pdfwrite',
        '-dCompatibilityLevel=1.7',
        '-dDetectDuplicateImages=true',
        '-dCompressFonts=true',
        '-dSubsetFonts=true',
        '-dAutoFilterColorImages=false',
        '-dColorImageFilter=/DCTEncode',
        '-dJPEGQ=82',
        '-dDownsampleColorImages=true',
        '-dColorImageDownsampleType=/Bicubic',
        '-dColorImageDownsampleThreshold=1.0',
        '-dColorImageResolution=150',
        '-dAutoFilterGrayImages=false',
        '-dGrayImageFilter=/DCTEncode',
        '-dDownsampleGrayImages=true',
        '-dGrayImageDownsampleType=/Bicubic',
        '-dGrayImageDownsampleThreshold=1.0',
        '-dGrayImageResolution=150',
        '-dDownsampleMonoImages=true',
        '-dMonoImageDownsampleType=/Subsample',
        '-dMonoImageDownsampleThreshold=1.0',
        '-dMonoImageResolution=300',
        "-sOutputFile=$candidatePdf",
        $rawPdf
    )
    Invoke-Checked -Executable $ghostscript -Arguments $ghostscriptArguments

    if (-not (Test-Path -LiteralPath $candidatePdf -PathType Leaf)) {
        throw 'PDF compression did not produce the candidate PDF.'
    }

    $candidate = Get-Item -LiteralPath $candidatePdf
    $maximumSizeBytes = 50MB
    if ($candidate.Length -gt $maximumSizeBytes) {
        throw "Generated PDF is $([math]::Round($candidate.Length / 1MB, 2)) MiB; the limit is 50 MiB."
    }

    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $pdfInformation = (& $pdfinfo $candidatePdf 2>$null) -join "`n"
        $pdfInfoExitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }
    if ($pdfInfoExitCode -ne 0) {
        throw "pdfinfo could not validate the generated PDF.`n$pdfInformation"
    }

    $pageMatch = [regex]::Match($pdfInformation, '(?m)^Pages:\s+(\d+)\s*$')
    if (-not $pageMatch.Success -or [int]$pageMatch.Groups[1].Value -lt 2) {
        throw 'The generated PDF has no valid multi-page page count.'
    }
    $pageCount = [int]$pageMatch.Groups[1].Value

    $renderPrefix = Join-Path $renderDirectory 'README'
    Invoke-Checked -Executable $pdftoppm -Arguments @(
        '-png',
        '-r', '110',
        $candidatePdf,
        $renderPrefix
    )

    $renderedPageCount = (Get-ChildItem -LiteralPath $renderDirectory -Filter 'README-*.png' -File).Count
    if ($renderedPageCount -ne $pageCount) {
        throw "Rendered $renderedPageCount pages, but pdfinfo reported $pageCount pages."
    }

    Copy-Item -LiteralPath $candidatePdf -Destination $publishedPdf -Force

    Write-Host "Generated README.pdf"
    Write-Host "Pages: $pageCount"
    Write-Host "Size: $([math]::Round($candidate.Length / 1MB, 2)) MiB"
    Write-Host "Rendered pages: $renderDirectory"
}
finally {
    Pop-Location
}
