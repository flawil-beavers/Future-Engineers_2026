local print_widths = {
  ["Component"] = { 0.31, 0.12, 0.43, 0.14 },
  ["Artifact"] = { 0.30, 0.70 },
  ["Specification"] = { 0.56, 0.44 },
  ["Stage and gearbox"] = { 0.22, 0.19, 0.15, 0.44 },
  ["Factor"] = { 0.18, 0.82 },
  ["Symbol"] = { 0.25, 0.75 },
  ["Load"] = { 0.38, 0.20, 0.21, 0.21 },
  ["Regulator"] = { 0.22, 0.18, 0.16, 0.22, 0.22 },
  ["Function"] = { 0.33, 0.67 },
  ["Sensor"] = { 0.28, 0.72 },
  ["Firmware area"] = { 0.30, 0.70 },
  ["Subsystems"] = { 0.18, 0.35, 0.47 },
}

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
  local header = adjusted.head.rows[1]
  local first_heading = header and pandoc.utils.stringify(header.cells[1].contents)
  local widths = print_widths[first_heading]
  if widths and #widths ~= column_count then widths = nil end

  local engineering_evidence = column_count == 3
    and (first_heading == "Subsystems" or (
      first_heading == "Subsystem"
      and pandoc.utils.stringify(header.cells[2].contents) == "Test evidence"
      and pandoc.utils.stringify(header.cells[3].contents) == "Decision or improvement"))
  if engineering_evidence then
    -- This is not a BOM: both evidence and decisions contain long prose.
    widths = widths or { 0.18, 0.42, 0.40 }
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
  elseif not widths and column_count == 2 then
    widths = { 0.68, 0.32 }
  elseif not widths and column_count == 3 then
    widths = { 0.48, 0.14, 0.38 }
  end

  if widths then
    for index, width in ipairs(widths) do
      adjusted.colspecs[index][2] = width
    end
  end

  return adjusted
end
