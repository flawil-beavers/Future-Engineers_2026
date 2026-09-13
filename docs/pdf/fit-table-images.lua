-- Pandoc's default LaTeX image sizing uses the full text width. Inside the
-- README's three-column photo tables that makes each image wider than its
-- cell, clipping both neighbouring images and captions. Apply an explicit
-- print width only to images nested inside a table; ordinary figures retain
-- their natural Pandoc sizing.
function Table(table)
  local has_images = false
  local adjusted = pandoc.walk_block(table, {
    Image = function(image)
      has_images = true
      image.attributes.width = "1.75in"
      image.attributes.height = nil
      return image
    end,
  })

  if has_images then
    -- Explicit column widths make Pandoc emit wrapping paragraph columns
    -- instead of `lll`. Without them, captions make the second and third
    -- columns extend beyond the right edge of the PDF page.
    local column_width = 1 / #adjusted.colspecs
    for index = 1, #adjusted.colspecs do
      adjusted.colspecs[index][2] = column_width
    end

    return adjusted
  end

  -- GFM pipe tables otherwise become non-wrapping LaTeX columns. Give the
  -- descriptive columns most of the width and keep quantity/value columns
  -- compact so long BOM results remain inside the page margins.
  local column_count = #adjusted.colspecs
  local widths = nil
  local engineering_evidence = column_count == 3
    and adjusted.head.rows[1] ~= nil
    and pandoc.utils.stringify(adjusted.head.rows[1].cells[1].contents) == "Subsystem"
    and pandoc.utils.stringify(adjusted.head.rows[1].cells[2].contents) == "Test evidence"
    and pandoc.utils.stringify(adjusted.head.rows[1].cells[3].contents) == "Decision or improvement"
  if engineering_evidence then
    -- This is not a BOM: both evidence and decisions contain long prose.
    widths = { 0.18, 0.42, 0.40 }
    if FORMAT:match("latex") then
      local function align_row(row)
        for _, cell in ipairs(row.cells) do
          if #cell.contents > 0 then
            cell.contents:insert(1, pandoc.RawBlock("latex", "\\raggedright"))
          end
        end
      end
      for _, row in ipairs(adjusted.head.rows) do align_row(row) end
      for _, body in ipairs(adjusted.bodies) do
        for _, row in ipairs(body.head) do align_row(row) end
        for _, row in ipairs(body.body) do align_row(row) end
      end
    end
  elseif column_count == 2 then
    widths = { 0.68, 0.32 }
  elseif column_count == 3 then
    widths = { 0.48, 0.14, 0.38 }
  end

  if widths then
    for index, width in ipairs(widths) do
      adjusted.colspecs[index][2] = width
    end
  end

  return adjusted
end
