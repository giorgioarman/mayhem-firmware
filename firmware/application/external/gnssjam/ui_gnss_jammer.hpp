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

 #ifndef __UI_GNSS_JAMMER_HPP
#define __UI_GNSS_JAMMER_HPP

#include "app_settings.hpp"
#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_language.hpp"
#include "string_format.hpp"
#include "transmitter_model.hpp"
#include "radio_state.hpp"
#include "replay_thread.hpp"
#include "sd_card.hpp"
#include "ui_transmitter.hpp"
#include <string>
#include <memory>


namespace ui::external_app::gnssjam {

class GnssJammerView : public View {
    public:
        GnssJammerView(NavigationView& nav);
        ~GnssJammerView();
        std::string title() const override { 
            return "GNSS Jammer"; 
        };
        void focus() override;

    private:
        NavigationView& nav_;
        //TxRadioState radio_state_{};
        TxRadioState radio_state_{
            1575420000 /* frequency */,
            20'000'000 /* bandwidth */,
            20'000'000 /* sampling rate */
        };
        app_settings::SettingsManager settings_{
            "tx_gnss_jammer", app_settings::Mode::TX};


        uint32_t center_freq = 1575420000;
        std::string selected_jammer = "LWF";
        u_int8_t tx_gain = 17;
        std::string iq_file_path;

        const size_t read_size{8192};
        const size_t buffer_count{2};

        const Style& style_val = *Theme::getInstance()->fg_green;
        const Style& style_cancel = *Theme::getInstance()->fg_red;

        std::unique_ptr<ReplayThread> replay_thread{};
        bool is_transmitting{false};
        bool ready_signal{false};
        


        bool check_sd_card(); // check if the SD card is mounted
        void toggle();
        void start_tx();  // Start transmission
        void stop_tx(const bool do_loop);   // Stop transmission
        void set_ready();
        void handle_replay_thread_done(const uint32_t return_code);

        MessageHandlerRegistration message_handler_replay_thread_error{
            Message::ID::ReplayThreadDone,
            [this](const Message* const p) {
                const auto message = *reinterpret_cast<const ReplayThreadDoneMessage*>(p);
                this->handle_replay_thread_done(message.return_code);
            }};

        MessageHandlerRegistration message_handler_fifo_signal{
            Message::ID::RequestSignal,
            [this](const Message* const p) {
                const auto message = static_cast<const RequestSignalMessage*>(p);
                if (message->signal == RequestSignalMessage::Signal::FillRequest) {
                    this->set_ready();
                }
            }};
        

        // UI Elements
        Labels labels{
            {{3 * 8, 2 * 8}, "GNSS Band:", Color::light_grey()},
            {{1 * 8, 5 * 8}, "Jammer Type:", Color::light_grey()},
            {{5 * 8, 8 * 8}, "TX Gain:", Color::light_grey()}, 
            {{8 * 8, 36 * 8}, "LINKS - NavSAS", Color::white()},
        };

        OptionsField options_gnss_band{
            {14 * 8, 2 * 8},
            3,
            {
                {"L1", 0},
                {"L2", 1},
                {"L5", 2}
            }
        };

        OptionsField options_jammer_type{
            {14 * 8, 5 * 8},
            5,
            {
                {"LWF", 0},
                {"LN", 1},
                {"TRI", 2},
                {"TW", 3},
                {"TICK", 4}
            }
        };

        NumberField options_tx_gain{
            {14 * 8, 8 * 8},
            2,
            {0, 47},
            1,
            ' ',
        };    

        Button button_transmit{
            {8 * 8, 11 * 8, 110, 50},
            "Start!"};

        Console log_console{
            {1, 18 * 8, 238, 120}
        };

        LiveDateTime timestamp{
            {5 * 8, 34 * 8, 19 * 8, 20}  // Positioned at the bottom of the screen
        };
};

} // namespace ui::external_app::gnss_jammer

#endif // __UI_GNSS_JAMMER_HPP
