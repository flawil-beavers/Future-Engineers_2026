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
  if column_count == 2 then
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
