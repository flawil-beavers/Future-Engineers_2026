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
