/*
 * Copyright (C) 2023 Bernd Herzog
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui.hpp"
#include "ui_gnss_jammer.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::gnssjam {
void initialize_app(ui::NavigationView& nav) {
    nav.push<GnssJammerView>();
}
}  // namespace ui::external_app::gnssjam

extern "C" {

__attribute__((section(".external_app.app_gnssjam.application_information"), used)) application_information_t _application_information_gnssjam = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::gnssjam::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "GnssJam",
    /*.bitmap_data = */ {
        0xC0,
        0x07,
        0xE0,
        0x0F,
        0x70,
        0x1F,
        0x78,
        0x3E,
        0x78,
        0x3C,
        0x78,
        0x38,
        0x78,
        0x30,
        0x78,
        0x38,
        0x78,
        0x3C,
        0x70,
        0x1E,
        0x70,
        0x1F,
        0xE0,
        0x0F,
        0xC0,
        0x07,
        0x80,
        0x03,
        0x20,
        0x09,
        0x50,
        0x14,
    },
    /*.icon_color = */ ui::Color::orange().v,
    /*.menu_location = */ app_location_t::TX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = portapack::spi_flash::image_tag_gnssjam */ {'P', 'G', 'J', 'M'},
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
