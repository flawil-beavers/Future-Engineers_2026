-- QR codes are generated as vectors by XeLaTeX, only for PDF/LaTeX output.
-- Video links in README.md are the source of truth; no remote QR service.
local function youtube_url(url)
  local id = url:match('^https://youtu%.be/([%w_-]+)')
  if not id and url:match('^https://www%.youtube%.com/watch%?') then
    id = url:match('[?&]v=([%w_-]+)')
  end
  if not id then
    id = url:match('^https://www%.youtube%.com/shorts/([%w_-]+)')
  end
  if id then return 'https://youtu.be/' .. id end
end

local function qr_block(url, label)
  return '\\begin{center}\n\\begin{minipage}{0.95\\linewidth}\n'
    .. '\\centering\n\\textbf{' .. label .. '}\\par\\medskip\n'
    .. '\\DocumentationQr{' .. url .. '}\\par\\medskip\n'
    .. '{\\small\\url{' .. url .. '}}\n'
    .. '\\end{minipage}\n\\end{center}\n'
end

function Pandoc(doc)
  if not FORMAT:match('latex') then return doc end

  local repository = pandoc.utils.stringify(doc.meta['repository-url'] or '')
  if not repository:match('^https://github%.com/[%w_.-]+/[%w_.-]+$') then
    error('Set repository-url to the canonical https://github.com/owner/repository URL.')
  end
  local includes = doc.meta['header-includes'] or pandoc.MetaList({})
  includes:insert(pandoc.MetaBlocks({pandoc.RawBlock('latex',
    '\\usepackage{graphicx}\n\\usepackage{qrcode}\n\\input{docs/pdf/qr-layout.tex}\n'
      .. '\\makeatletter\n\\newcommand{\\DocumentationQr}[1]{\\begingroup\\color{black}'
      .. '\\let\\DocumentationRestore\\@minipagerestore'
      .. '\\def\\@minipagerestore{\\DocumentationRestore\\parskip=0pt\\lineskip=0pt'
      .. '\\lineskiplimit=0pt\\parindent=0pt\\leftskip=0pt\\rightskip=0pt\\parfillskip=0pt}'
      .. '\\qrcode[height=28mm,level=M,nolink,padding]{#1}\\endgroup}\n\\makeatother\n'
      .. '\\newcommand{\\DocumentationRepositoryQr}{'
      .. qr_block(repository, 'Source code and engineering documentation') .. '}')}))
  doc.meta['header-includes'] = includes

  local blocks = pandoc.List({})
  local in_video = false
  local video_level = nil
  local label = 'Watch the video'
  local seen = {}
  for _, block in ipairs(doc.blocks) do
    if block.t == 'Header' then
      local title = pandoc.utils.stringify(block.content)
      if title == 'Video' then
        in_video = true
        video_level = block.level
      elseif in_video and block.level <= video_level then
        in_video = false
      elseif in_video then
        if title:lower():match('open challenge') then
          label = 'Open Challenge video'
        elseif title:lower():match('obstacle challenge') then
          label = 'Obstacle Challenge video'
        else
          label = 'Watch the video'
        end
      end
    end
    blocks:insert(block)
    if in_video then
      local urls = {}
      block:walk({Link = function(link)
        local url = youtube_url(link.target)
        if url and not seen[url] then
          seen[url] = true
          table.insert(urls, url)
        end
      end})
      for _, url in ipairs(urls) do
        blocks:insert(pandoc.RawBlock('latex', qr_block(url, label)))
      end
    end
  end
  doc.blocks = blocks
  return doc
end
