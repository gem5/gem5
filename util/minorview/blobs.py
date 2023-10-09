# Copyright (c) 2013 ARM Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# blobs.py: Blobs are the visual blocks, arrows and other coloured
#   objects on the visualiser.  This file contains Blob definition and
#   their rendering instructions in pygtk/cairo.
#
import pygtk

pygtk.require("2.0")
import gtk
import gobject
import cairo
import re
import math

from .point import Point
from . import parse
from . import colours
from .colours import backgroundColour, black
from . import model


def centre_size_to_sides(centre, size):
    """Returns a 4-tuple of the relevant ordinates of the left,
    right, top and bottom sides of the described rectangle"""
    (x, y) = centre.to_pair()
    (half_width, half_height) = (size.scale(0.5)).to_pair()
    left = x - half_width
    right = x + half_width
    top = y - half_height
    bottom = y + half_height
    return (left, right, top, bottom)


def box(cr, centre, size):
    """Draw a simple box"""
    (left, right, top, bottom) = centre_size_to_sides(centre, size)
    cr.move_to(left, top)
    cr.line_to(right, top)
    cr.line_to(right, bottom)
    cr.line_to(left, bottom)
    cr.close_path()


def stroke_and_fill(cr, colour):
    """Stroke with the current colour then fill the same path with the
    given colour"""
    join = cr.get_line_join()
    cr.set_line_join(gtk.gdk.JOIN_ROUND)
    cr.close_path()
    cr.set_source_color(backgroundColour)
    cr.stroke_preserve()
    cr.set_source_color(colour)
    cr.fill()
    cr.set_line_join(join)


def striped_box(cr, centre, size, colours):
    """Fill a rectangle (without outline) striped with the colours given"""
    num_colours = len(colours)
    if num_colours == 0:
        box(cr, centre, size)
        cr.set_source_color(backgroundColour)
        cr.fill()
    elif num_colours == 1:
        box(cr, centre, size)
        stroke_and_fill(cr, colours[0])
    else:
        (left, right, top, bottom) = centre_size_to_sides(centre, size)
        (width, height) = size.to_pair()
        x_stripe_width = width / num_colours
        half_x_stripe_width = x_stripe_width / 2.0
        # Left triangle
        cr.move_to(left, bottom)
        cr.line_to(left + half_x_stripe_width, bottom)
        cr.line_to(left + x_stripe_width + half_x_stripe_width, top)
        cr.line_to(left, top)
        stroke_and_fill(cr, colours[0])
        # Stripes
        for i in range(1, num_colours - 1):
            xOffset = x_stripe_width * i
            cr.move_to(left + xOffset - half_x_stripe_width, bottom)
            cr.line_to(left + xOffset + half_x_stripe_width, bottom)
            cr.line_to(
                left + xOffset + x_stripe_width + half_x_stripe_width, top
            )
            cr.line_to(
                left + xOffset + x_stripe_width - half_x_stripe_width, top
            )
            stroke_and_fill(cr, colours[i])
        # Right triangle
        cr.move_to((right - x_stripe_width) - half_x_stripe_width, bottom)
        cr.line_to(right, bottom)
        cr.line_to(right, top)
        cr.line_to((right - x_stripe_width) + half_x_stripe_width, top)
        stroke_and_fill(cr, colours[num_colours - 1])


def speech_bubble(cr, top_left, size, unit):
    """Draw a speech bubble with 'size'-sized internal space with its
    top left corner at Point(2.0 * unit, 2.0 * unit)"""

    def local_arc(centre, angleFrom, angleTo):
        cr.arc(
            centre.x, centre.y, unit, angleFrom * math.pi, angleTo * math.pi
        )

    cr.move_to(*top_left.to_pair())
    cr.rel_line_to(unit * 2.0, unit)
    cr.rel_line_to(size.x, 0.0)
    local_arc(top_left + Point(size.x + unit * 2.0, unit * 2.0), -0.5, 0.0)
    cr.rel_line_to(0.0, size.y)
    local_arc(
        top_left + Point(size.x + unit * 2.0, size.y + unit * 2.0), 0, 0.5
    )
    cr.rel_line_to(-size.x, 0.0)
    local_arc(top_left + Point(unit * 2.0, size.y + unit * 2.0), 0.5, 1.0)
    cr.rel_line_to(0, -size.y)
    cr.close_path()


def open_bottom(cr, centre, size):
    """Draw a box with left, top and right sides"""
    (left, right, top, bottom) = centre_size_to_sides(centre, size)
    cr.move_to(left, bottom)
    cr.line_to(left, top)
    cr.line_to(right, top)
    cr.line_to(right, bottom)


def fifo(cr, centre, size):
    """Draw just the vertical sides of a box"""
    (left, right, top, bottom) = centre_size_to_sides(centre, size)
    cr.move_to(left, bottom)
    cr.line_to(left, top)
    cr.move_to(right, bottom)
    cr.line_to(right, top)


def cross(cr, centre, size):
    """Draw a cross parallel with the axes"""
    (left, right, top, bottom) = centre_size_to_sides(centre, size)
    (x, y) = centre.to_pair()
    cr.move_to(left, y)
    cr.line_to(right, y)
    cr.move_to(x, top)
    cr.line_to(x, bottom)


class Blob(object):
    """Blob super class"""

    def __init__(self, picChar, unit, topLeft, colour, size=Point(1, 1)):
        self.picChar = picChar
        self.unit = unit
        self.displayName = unit
        self.nameLoc = "top"
        self.topLeft = topLeft
        self.colour = colour
        self.size = size
        self.border = 1.0
        self.dataSelect = model.BlobDataSelect()
        self.shorten = 0

    def render(self, cr, view, event, select, time):
        """Render this blob with the given event's data.  Returns either
        None or a pair of (centre, size) in device coordinates for the drawn
        blob.  The return value can be used to detect if mouse clicks on
        the canvas are within the blob"""
        return None


class Block(Blob):
    """Blocks are rectangular blogs colourable with a 2D grid of striped
    blocks.  visualDecoder specifies how event data becomes this coloured
    grid"""

    def __init__(
        self,
        picChar,
        unit,
        topLeft=Point(0, 0),
        colour=colours.black,
        size=Point(1, 1),
    ):
        super(Block, self).__init__(picChar, unit, topLeft, colour, size=size)
        # {horiz, vert}
        self.stripDir = "horiz"
        # {LR, RL}: LR means the first strip will be on the left/top,
        #   RL means the first strip will be on the right/bottom
        self.stripOrd = "LR"
        # Number of blank strips if this is a frame
        self.blankStrips = 0
        # {box, fifo, openBottom}
        self.shape = "box"
        self.visualDecoder = None

    def render(self, cr, view, event, select, time):
        # Find the right event, visuals and sizes for things
        if event is None or self.displayName.startswith("_"):
            event = model.BlobEvent(self.unit, time)

        if self.picChar in event.visuals:
            strips = event.visuals[self.picChar].to_striped_block(
                select & self.dataSelect
            )
        else:
            strips = [[[colours.unknownColour]]]

        if self.stripOrd == "RL":
            strips.reverse()

        if len(strips) == 0:
            strips = [[colours.errorColour]]
            print("Problem with the colour of event:", event)

        num_strips = len(strips)
        strip_proportion = 1.0 / num_strips
        first_strip_offset = (num_strips / 2.0) - 0.5

        # Adjust blocks with 'shorten' attribute to the length of the data
        size = Point(*self.size.to_pair())
        if self.shorten != 0 and self.size.x > (num_strips * self.shorten):
            size.x = num_strips * self.shorten

        box_size = size - view.blobIndentFactor.scale(2)

        # Now do cr sensitive things
        cr.save()
        cr.scale(*view.pitch.to_pair())
        cr.translate(*self.topLeft.to_pair())
        cr.translate(*(size - Point(1, 1)).scale(0.5).to_pair())

        translated_centre = Point(*cr.user_to_device(0.0, 0.0))
        translated_size = Point(*cr.user_to_device_distance(*size.to_pair()))

        # The 2D grid is a grid of strips of blocks.  Data [[1,2],[3]]
        # is 2 strips of 2 and 1 blocks respectively.
        # if stripDir == 'horiz', strips are stacked vertically
        #   from top to bottom if stripOrd == 'LR' or bottom to top if
        #   stripOrd == 'RL'.
        # if stripDir == 'vert', strips are stacked horizontally
        #   from left to right if stripOf == 'LR' or right to left if
        #   stripOrd == 'RL'.

        strip_is_horiz = self.stripDir == "horiz"

        if strip_is_horiz:
            strip_step_base = Point(1.0, 0.0)
            block_step_base = Point(0.0, 1.0)
        else:
            strip_step_base = Point(0.0, 1.0)
            block_step_base = Point(1.0, 0.0)

        strip_size = box_size * (
            strip_step_base.scale(strip_proportion) + block_step_base
        )
        strip_step = strip_size * strip_step_base
        strip_centre = Point(0, 0) - (
            strip_size * strip_step_base.scale(first_strip_offset)
        )

        cr.set_line_width(view.midLineWidth / view.pitch.x)

        # Draw the strips and their blocks
        for strip_index in range(num_strips):
            num_blocks = len(strips[strip_index])
            block_proportion = 1.0 / num_blocks
            firstBlockOffset = (num_blocks / 2.0) - 0.5

            block_size = strip_size * (
                block_step_base.scale(block_proportion) + strip_step_base
            )
            block_step = block_size * block_step_base
            block_centre = (
                strip_centre
                + strip_step.scale(strip_index)
                - (block_size * block_step_base.scale(firstBlockOffset))
            )

            for block_index in range(num_blocks):
                striped_box(
                    cr,
                    block_centre + block_step.scale(block_index),
                    block_size,
                    strips[strip_index][block_index],
                )

        cr.set_font_size(0.7)
        if self.border > 0.5:
            weight = cairo.FONT_WEIGHT_BOLD
        else:
            weight = cairo.FONT_WEIGHT_NORMAL
        cr.select_font_face("Helvetica", cairo.FONT_SLANT_NORMAL, weight)

        xb, yb, width, height, dx, dy = cr.text_extents(self.displayName)

        text_comfort_space = 0.15

        if self.nameLoc == "left":
            # Position text vertically along left side, top aligned
            cr.save()
            cr.rotate(-(math.pi / 2.0))
            text_point = Point(size.y, size.x).scale(0.5) * Point(-1, -1)
            text_point += Point(max(0, size.y - width), 0)
            text_point += Point(-text_comfort_space, -text_comfort_space)
        else:  # Including top
            # Position text above the top left hand corner
            text_point = size.scale(0.5) * Point(-1, -1)
            text_point += Point(0.00, -text_comfort_space)

        if self.displayName != "" and not self.displayName.startswith("_"):
            cr.set_source_color(self.colour)
            cr.move_to(*text_point.to_pair())
            cr.show_text(self.displayName)

        if self.nameLoc == "left":
            cr.restore()

        # Draw the outline shape
        cr.save()
        if strip_is_horiz:
            cr.rotate(-(math.pi / 2.0))
            box_size = Point(box_size.y, box_size.x)

        if self.stripOrd == "RL":
            cr.rotate(math.pi)

        if self.shape == "box":
            box(cr, Point(0, 0), box_size)
        elif self.shape == "openBottom":
            open_bottom(cr, Point(0, 0), box_size)
        elif self.shape == "fifo":
            fifo(cr, Point(0, 0), box_size)
        cr.restore()

        # Restore scale and stroke the outline
        cr.restore()
        cr.set_source_color(self.colour)
        cr.set_line_width(view.thickLineWidth * self.border)
        cr.stroke()

        # Return blob size/position
        if self.unit == "_":
            return None
        else:
            return (translated_centre, translated_size)


class Key(Blob):
    """Draw a key to the special (and numeric colours) with swatches of the
    colours half as wide as the key"""

    def __init__(
        self, picChar, unit, topLeft, colour=colours.black, size=Point(1, 1)
    ):
        super(Key, self).__init__(picChar, unit, topLeft, colour, size=size)
        self.colours = "BBBB"
        self.displayName = unit

    def render(self, cr, view, event, select, time):
        cr.save()
        cr.scale(*view.pitch.to_pair())
        cr.translate(*self.topLeft.to_pair())
        # cr.translate(*(self.size - Point(1,1)).scale(0.5).to_pair())
        half_width = self.size.x / 2.0
        cr.translate(
            *(self.size - Point(1.0 + half_width, 1.0)).scale(0.5).to_pair()
        )

        num_colours = len(self.colours)
        cr.set_line_width(view.midLineWidth / view.pitch.x)

        blob_size = Point(half_width, 0.0) + (
            self.size * Point(0.0, 1.0 / num_colours)
        )
        blob_step = Point(0.0, 1.0) * blob_size
        first_blob_centre = Point(0.0, 0.0) - blob_step.scale(
            (num_colours / 2.0) - 0.5
        )

        cr.set_source_color(self.colour)
        cr.set_line_width(view.thinLineWidth / view.pitch.x)

        blob_proportion = 0.8

        real_blob_size = blob_size.scale(blob_proportion)

        cr.set_font_size(0.8 * blob_size.y * blob_proportion)
        cr.select_font_face(
            "Helvetica", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD
        )

        for i in range(num_colours):
            centre = first_blob_centre + blob_step.scale(i)
            box(cr, centre, real_blob_size)

            colour_char = self.colours[i]
            if colour_char.isdigit():
                cr.set_source_color(colours.number_to_colour(int(colour_char)))
                label = "..." + colour_char
            else:
                cr.set_source_color(model.special_state_colours[colour_char])
                label = model.special_state_names[colour_char]

            cr.fill_preserve()
            cr.set_source_color(self.colour)
            cr.stroke()

            xb, yb, width, height, dx, dy = cr.text_extents(label)

            text_left = (
                centre
                + (Point(0.5, 0.0) * blob_size)
                + Point(0.0, height / 2.0)
            )

            cr.move_to(*text_left.to_pair())
            cr.show_text(label)


class Arrow(Blob):
    """Draw a left or right facing arrow"""

    def __init__(
        self,
        unit,
        topLeft,
        colour=colours.black,
        size=Point(1.0, 1.0),
        direc="right",
    ):
        super(Arrow, self).__init__(unit, unit, topLeft, colour, size=size)
        self.direc = direc

    def render(self, cr, view, event, select, time):
        cr.save()
        cr.scale(*view.pitch.to_pair())
        cr.translate(*self.topLeft.to_pair())
        cr.translate(*(self.size - Point(1, 1)).scale(0.5).to_pair())
        cr.scale(*self.size.to_pair())
        (blob_indent_x, blob_indent_y) = (
            view.blobIndentFactor / self.size
        ).to_pair()
        left = -0.5 - blob_indent_x
        right = 0.5 + blob_indent_x

        thickness = 0.2
        flare = 0.2

        if self.direc == "left":
            cr.rotate(math.pi)

        cr.move_to(left, -thickness)
        cr.line_to(0.0, -thickness)
        cr.line_to(0.0, -(thickness + flare))
        cr.line_to(right, 0)
        # Break arrow to prevent the point ruining the appearance of boxes
        cr.move_to(right, 0)
        cr.line_to(0.0, (thickness + flare))
        cr.line_to(0.0, +thickness)
        cr.line_to(left, +thickness)

        cr.restore()

        # Draw arrow a bit more lightly than the standard line width
        cr.set_line_width(cr.get_line_width() * 0.75)
        cr.set_source_color(self.colour)
        cr.stroke()

        return None
