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
    {"TRIW", "Triangular Wave"},
    {"TICK", "TICK"}
};

namespace ui 
{
    // Focus function (sets default selection)
    void GnssJammerView::focus() {
        button_transmit.focus();
    }

    void GnssJammerView::change_back_button(){
        if (is_transmitting){
            button_transmit.set_text("STOP");
            button_transmit.set_style(&style_cancel);
        } else{
            button_transmit.set_text("START");
            button_transmit.set_style(&style_start);
        }
    }

    void GnssJammerView::change_button(){
        button_transmit.set_text("RESTART");
        button_transmit.set_style(&style_restart);
    }

    // Constructor
    GnssJammerView::GnssJammerView(NavigationView& nav): nav_(nav)
    {
        // Update the sample rate in proc_replay baseband.
        baseband::run_image(portapack::spi_flash::image_tag_gnssjam);
        // baseband::run_prepared_image(portapack::memory::map::m4_code.base());

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
        log_console.writeln("App started");
        log_console.set_style(Theme::getInstance()->bg_light);

        // Set button behavior
        button_transmit.on_select = [this](Button &) {
            this->toggle();
        };
        button_transmit.set_style(&style_start);
        
        // When options_tx_gain is changed
        options_tx_gain.on_change = [this](int_fast8_t v) {
            if(is_transmitting){
                log_console.write("\nTX gain is set: " + std::to_string(static_cast<int>(v)) + " dB");
                transmitter_model.set_tx_gain(v);
            }
        };

        // When options_tx_gain is changed
        options_jammer_type.on_change = [this](size_t, OptionsField::value_t v) {
            if(is_transmitting){
                if (options_jammer_type.selected_index_name()!= selected_jammer)
                    change_button();
                else
                    change_back_button();        
            }
        };

        // When options_tx_gain is changed
        options_gnss_band.on_change = [this](size_t, OptionsField::value_t v) {
            if(is_transmitting){
                if (options_gnss_band.selected_index_name()!= selected_band)
                    change_button();
                else
                    change_back_button(); 
            }
        };

        transmitter_model.set_sampling_rate(20000000);
        transmitter_model.set_baseband_bandwidth(20000000);
        transmitter_model.set_rf_amp(true); 
        options_tx_gain.set_value(tx_gain);

        // Enable seconds in the timestamp display
        timestamp.set_seconds_enabled(true);  
        
    }

    GnssJammerView::~GnssJammerView() {
        // transmitter_model.disable();
        stop_tx(false);
        baseband::shutdown();
    }

    void GnssJammerView::toggle() {
        button_transmit.focus();
        if (is_transmitting) {
            if (button_transmit.text() == "RESTART")
            {
                if ((bool)replay_thread)
                    replay_thread.reset();
                is_transmitting = false;
                start_tx();
            } else
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
            log_console.write("\nStart Jamming");
            // Check SD Card
            if(!check_sd_card()) { // Check to see if SD Card is mounted
                log_console.write("\nError: SD card is not mounted.");
                return;
            } else{
                log_console.write("\nSD card is mounted.");
            }

            std::string selected_band = options_gnss_band.selected_index_name();
            // Default L1
            if (selected_band == "L2") {
                center_freq = 1227600000;
            } else if (selected_band == "L5") {
                center_freq = 1176450000;
            }
            log_console.write("\nCenter freq: " + std::to_string(center_freq) + " Hz");

            tx_gain = options_tx_gain.value();
            log_console.write("\nTX gain: " + std::to_string(tx_gain) + " dB");

            selected_jammer = options_jammer_type.selected_index_name();
            std::string jammerTypeFullName = "Unknown";  // Default value
            auto it = jammerTypeMap.find(selected_jammer);
            if (it != jammerTypeMap.end()) {
                jammerTypeFullName = it->second;
            }
            log_console.write("\nJammer type: " + jammerTypeFullName);
            iq_file_path = "/GNSS_JAMMER/BIN_FILES/" + selected_jammer + ".C8";

            transmitter_model.set_target_frequency(center_freq);
        }
        
        // Check SD Card
        if(!check_sd_card()) { // Check to see if SD Card is mounted
            log_console.write("\nError: SD card is not mounted.");
            return;
        }

        std::unique_ptr<stream::Reader> reader;

        auto p = std::make_unique<FileReader>();
        auto open_error = p->open(iq_file_path);
        if (open_error.is_valid()) {
            log_console.write("\nError: Unable to read IQ file.");
            return;
        } else {
            reader = std::move(p);
        }
        
        if (reader) {
            if (!is_transmitting){
                button_transmit.set_style(&style_cancel);
                button_transmit.set_text("STOP");
            }
    
            replay_thread = std::make_unique<ReplayThread>(
                std::move(reader),
                read_size, buffer_count,
                &ready_signal,
                [](uint32_t return_code) {
                    ReplayThreadDoneMessage message{return_code};
                    EventDispatcher::send_message(message);
                });
            transmitter_model.enable();
            is_transmitting = true;
            counter++;
            if (counter % 10 == 0) {
                log_console.write(".");
            }
        }
    }

    void GnssJammerView::stop_tx(const bool do_loop)
    {
        if ((bool)replay_thread)
            replay_thread.reset();

        if (do_loop) {
            start_tx();
        } else {
            log_console.write("\nstop Jamming!");
            button_transmit.set_text("START");
            button_transmit.set_style(&style_start);
            transmitter_model.disable();
            is_transmitting = false;
            counter =0;
        }
        ready_signal = false;
    }

    // Checks SD Card, returns true if Mounted, false if otherwise
    bool GnssJammerView::check_sd_card() {
        return (sd_card::status() == sd_card::Status::Mounted) ? true : false; 
    }

} // namespace ui::external_app::gnss_jammer
