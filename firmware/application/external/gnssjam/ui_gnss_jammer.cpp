/*
 * Copyright (C) 2016 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
 * Copyright (C) 2020 Shao
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

#include "ui_gnss_jammer.hpp"
#include "baseband_api.hpp"
#include "convert.hpp"
#include "file_reader.hpp"
#include "io_file.hpp"
#include "io_convert.hpp"
#include "oversample.hpp"
// #include "portapack.hpp"
#include "portapack_persistent_memory.hpp"
#include "string_format.hpp"
#include "ui_fileman.hpp"
#include "utility.hpp"
#include "file_path.hpp"

#include <unistd.h>
#include <fstream>

using namespace portapack;

const std::unordered_map<std::string, std::string> jammerTypeMap = {
    {"LWF", "Linear Wide Fast"},
    {"LN", "Linear Narrow"},
    {"TRI", "Triangular"},
    {"TW", "Triangular Wave"},
    {"TICK", "TICK"}
};

namespace ui ::external_app::gnssjam
{
    // Focus function (sets default selection)
    void GnssJammerView::focus() {
        button_transmit.focus();
    }

    // Constructor
    GnssJammerView::GnssJammerView(NavigationView& nav): nav_(nav)
    {
        // Update the sample rate in proc_replay baseband.
        baseband::run_image(portapack::spi_flash::image_tag_gnssjam);

        // Add all UI components to the view
        add_children({
            &labels,
            &options_jammer_type,
            &options_gnss_band,
            &options_tx_gain,
            &button_transmit,
            &log_console,
            &timestamp  // Add timestamp at the bottom
        });
        log_console.enable_scrolling(false);
        log_console.clear(true);
        log_console.write("App started \n");
        log_console.set_style(Theme::getInstance()->bg_light);

        // Set button behavior
        button_transmit.on_select = [this](Button &) {
            this->toggle();
        };
        button_transmit.set_style(&style_val);
        
        // When options_tx_gain is changed
        options_tx_gain.on_change = [this](int_fast8_t v) {
            log_console.writeln("TX gain is set: " + std::to_string(static_cast<int>(v)) + " dB");
            transmitter_model.set_tx_gain(v);
        };

        transmitter_model.set_sampling_rate(20000000);
        transmitter_model.set_baseband_bandwidth(20000000);
        transmitter_model.set_rf_amp(true); 

        // Enable seconds in the timestamp display
        timestamp.set_seconds_enabled(true);  
        
    }

    GnssJammerView::~GnssJammerView() {
        transmitter_model.disable();
        baseband::shutdown();
    }

    void GnssJammerView::toggle() {
        if (is_transmitting) {
            stop_tx(false);
        } else {
            start_tx();
        }
    }

    void GnssJammerView::set_ready() {
        ready_signal = true;
    }

    void GnssJammerView::handle_replay_thread_done(const uint32_t return_code) {
        if (return_code == ReplayThread::END_OF_FILE) {
            stop_tx(true);
        } else if (return_code == ReplayThread::READ_ERROR) {
            stop_tx(false);
        }
    }

    void GnssJammerView::start_tx()
    {
        // stop_tx(false);
        if (!is_transmitting){
            log_console.writeln("Start Jamming");
            // Check SD Card
            if(!check_sd_card()) { // Check to see if SD Card is mounted
                log_console.writeln("Error: SD card is not mounted.");
                return;
            } else{
                log_console.writeln("SD card is mounted.");
            }

            std::string selected_band = options_gnss_band.selected_index_name();
            // Default L1
            if (selected_band == "L2") {
                center_freq = 1227600000;
            } else if (selected_band == "L5") {
                center_freq = 1176450000;
            }
            log_console.writeln("Center freq: " + std::to_string(center_freq) + " Hz");

            tx_gain = options_tx_gain.value();
            log_console.writeln("TX gain: " + std::to_string(tx_gain) + " dB");

            selected_jammer = options_jammer_type.selected_index_name();
            std::string jammerTypeFullName = "Unknown";  // Default value
            auto it = jammerTypeMap.find(selected_jammer);
            if (it != jammerTypeMap.end()) {
                jammerTypeFullName = it->second;
            }
            log_console.writeln("Jammer type: " + jammerTypeFullName);
            iq_file_path = "/GNSS_JAMMER/BIN_FILES/" + selected_jammer + ".C8";

            transmitter_model.set_target_frequency(center_freq);
        }
        
        // Check SD Card
        if(!check_sd_card()) { // Check to see if SD Card is mounted
            log_console.writeln("Error: SD card is not mounted.");
            return;
        }

        std::unique_ptr<stream::Reader> reader;

        auto p = std::make_unique<FileReader>();
        auto open_error = p->open(iq_file_path);
        if (open_error.is_valid()) {
            log_console.writeln("Error: Unable to read IQ file.");
            return;
        } else {
            reader = std::move(p);
            log_console.writeln("Read IQ file.");
        }
        
        if (reader) {
            button_transmit.set_style(&style_cancel);
            button_transmit.set_text("STOP");
            button_transmit.focus();
            is_transmitting = true;
    
            replay_thread = std::make_unique<ReplayThread>(
                std::move(reader),
                read_size, buffer_count,
                &ready_signal,
                [](uint32_t return_code) {
                    ReplayThreadDoneMessage message{return_code};
                    EventDispatcher::send_message(message);
                });
            transmitter_model.enable();
        }
    }

    void GnssJammerView::stop_tx(const bool do_loop)
    {
        if ((bool)replay_thread)
            replay_thread.reset();

        if (do_loop) {
            start_tx();
        } else {
            log_console.writeln("stop Jamming!");
            button_transmit.set_text("Start!");
            button_transmit.set_style(&style_val);
            transmitter_model.disable();
            is_transmitting = false;
        }
        ready_signal = false;
    }

    // Checks SD Card, returns true if Mounted, false if otherwise
    bool GnssJammerView::check_sd_card() {
        return (sd_card::status() == sd_card::Status::Mounted) ? true : false; 
    }

} // namespace ui::external_app::gnss_jammer
