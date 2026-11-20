-- QR codes are generated locally, only for PDF/LaTeX output.
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
  local image_path = 'local_workspace/pdf-build/qr/' .. pandoc.utils.sha1(url) .. '.png'
  pandoc.pipe('python', {'scripts/render-documentation-qr.py', image_path}, url)
  return '\\begin{center}\n\\begin{minipage}{0.95\\linewidth}\n'
    .. '\\centering\n\\textbf{' .. label .. '}\\par\\medskip\n'
    .. '\\DocumentationQr{' .. image_path .. '}\\par\\medskip\n'
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
    '\\usepackage{graphicx}\n\\newcommand{\\DocumentationQr}[1]{\\includegraphics[width=33mm]{#1}}\n'
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
