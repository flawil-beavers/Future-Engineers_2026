-- Keep GitHub-friendly Unicode in README.md while producing a portable PDF
-- with the fonts supplied by a standard Pandoc/XeLaTeX installation.
function Str(text)
  text.text = text.text:gsub("≈", "approximately ")
  text.text = text.text:gsub("↔", "<->")
  text.text = text.text:gsub("🇨🇭", "(Switzerland)")
  return text
end

function CodeBlock(block)
  block.text = block.text:gsub("│", "|")
  block.text = block.text:gsub("├", "|--")
  block.text = block.text:gsub("└", "`--")
  block.text = block.text:gsub("─", "-")
  return block
end

function Header(header)
  -- The README nests sections under its title. Retain the print section sizes
  -- and keep third-level subsections from becoming run-in LaTeX headings.
  if FORMAT:match("latex") and header.level > 1 and header.identifier ~= "contents" then
    header.level = header.level - 1
  end
  if header.identifier == "bill-of-materials-bom" or header.identifier == "robot-photos" then
    return {pandoc.RawBlock("latex", "\\clearpage"), header}
  end
  return header
end

function Image(image)
  if image.src == "img/diagrams/Battery-holders-underside.jpg" then
    image.attributes.height = "2.5in"
  elseif image.src == "img/diagrams/Ackermann-construction.png" then
    image.attributes.height = "5.1in"
  elseif image.title == "Obstacle Challenge flow" and image.src:match("^%./diagrams%-%d+%.pdf$") then
    image.attributes.height = "7in"
  end
  return image
end
