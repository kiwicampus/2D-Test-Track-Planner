/*! @package speaker
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "modules/speaker.hpp"

Speaker::Speaker(rclcpp::NodeOptions &options) : Node("speaker", "interfaces", options)
{
    RCLCPP_DEBUG(this->get_logger(), "Speaker Contructor");

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // /* Publisher */
    m_done_pub = this->create_publisher<std_msgs::msg::Bool>("/device/speaker/done", default_qos);

    m_speaker_sub = this->create_subscription<std_msgs::msg::Int8>("/device/speaker/command", default_qos,
                                                                   std::bind(&Speaker::speakerCb, this, _1));

    /* Open the PCM device in playback mode */
    if (pcm = (snd_pcm_open(&pcm_handle, m_sound_device.c_str(), SND_PCM_STREAM_PLAYBACK, 0) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't open \"%s\" PCM device. %s", m_sound_device, snd_strerror(pcm));

    /* Allocate parameters object and fill it with default values*/
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);

    /* Set parameters */
    if (pcm = (snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set interleaved mode. %s", snd_strerror(pcm));

    if (pcm = (snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE) < 0))

        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set format. %s\n", snd_strerror(pcm));

    if (pcm = (snd_pcm_hw_params_set_channels(pcm_handle, params, channels) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set channels number. %s\n", snd_strerror(pcm));

    if (pcm = (snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, 0) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set rate. %s\n", snd_strerror(pcm));

    /* Write parameters */
    if (pcm = (snd_pcm_hw_params(pcm_handle, params) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set harware parameters. %s\n", snd_strerror(pcm));

    /* Resume information */
    RCLCPP_INFO(this->get_logger(), "PCM name: '%s'\n", snd_pcm_name(pcm_handle));

    RCLCPP_INFO(this->get_logger(), "PCM state: %s\n", snd_pcm_state_name(snd_pcm_state(pcm_handle)));

    snd_pcm_hw_params_get_period_size(params, &frames, 0);

    buff_size = frames * channels * 2;
    buff = (char *)malloc(buff_size);
    memset(buff, 0, buff_size);

    buff_size_ambient = frames * channels * 2;
    buff_ambient = (char *)malloc(buff_size_ambient);
    memset(buff_ambient, 0, buff_size_ambient);

    m_path = "/workspace/planner/media/audio/track";

    snd_pcm_start(pcm_handle);
    readfd_ambient = open((m_path + "0.wav").c_str(), O_RDONLY);
    ambient = pthread_create(&pthread_id_ambient, NULL, (THREADFUNCPTR)&Speaker::AmbientSound, this);
}

void Speaker::speakerCb(const std_msgs::msg::Int8::SharedPtr msg)
{
    /*
+       Note: Every sound-track should be named "track#"
        To Stop send a 0 message.
    */
    m_multi_sound = 0;
    snd_pcm_start(pcm_handle);
    if (msg->data > 0)
    {
        std::ifstream ifile;
        ifile.open(m_path + std::to_string(msg->data) + ".wav");
        if (ifile)
        {
            readfd = open((m_path + std::to_string(msg->data) + ".wav").c_str(), O_RDONLY);
        }
        else
        {
            readfd = open((m_path + "0.wav").c_str(), O_RDONLY);
            RCLCPP_WARN(this->get_logger(), "No sound found");
        }
        status = pthread_create(&pthread_id, NULL, (THREADFUNCPTR)&Speaker::PlaySound, this);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Sound stopped");
        m_pause = 1;
        m_multi_sound = 1;
    }
}

void *Speaker::PlaySound()
{
    if (readfd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "ERROR: .wav It's Empty");
        pthread_join(pthread_id, NULL);
    }
    std_msgs::msg::Bool::UniquePtr msg(new std_msgs::msg::Bool());
    msg->data = false;
    m_done_pub->publish(std::move(msg));
    while (readval = (read(readfd, buff, buff_size) > 0))
    {
        // if (pcm = snd_pcm_writei(pcm_handle, buff, readval) == -EPIPE)
        if (pcm = snd_pcm_writei(pcm_handle, buff, frames) == -EPIPE)
        {
            snd_pcm_prepare(pcm_handle);
        }
        else if (pcm < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Writing to PCM device: %s", snd_strerror(pcm));
        }
        if (m_pause)
        {
            snd_pcm_pause(pcm_handle, m_pause);
            m_pause = 0;
            break;
        }
    }
    m_multi_sound = 1;
    msg.reset(new std_msgs::msg::Bool());
    msg->data = true;
    m_done_pub->publish(std::move(msg));
    // snd_pcm_drain(pcm_handle);
}

void *Speaker::AmbientSound()
{
    while (true)
    {
        if (!m_pause)
        {
            readfd_ambient = open((m_path + "0.wav").c_str(), O_RDONLY);
            if (readfd_ambient < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR: .wav It's Empty");
                pthread_join(pthread_id_ambient, NULL);
            }
            while (readval_ambient = (read(readfd_ambient, buff_ambient, buff_size_ambient) > 0))
            {
                if (m_multi_sound)
                {
                    if (pcm = snd_pcm_writei(pcm_handle, buff_ambient, frames) == -EPIPE)
                    {
                        snd_pcm_prepare(pcm_handle);
                    }
                    else if (pcm < 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "ERROR: Writing to PCM device: %s", snd_strerror(pcm));
                    }
                    if (m_pause)
                    {
                        snd_pcm_pause(pcm_handle, m_pause);
                        m_pause = 0;
                        break;
                    }
                }
            }
        }
    }
}

/* Class Destructor to clean up everything*/
Speaker::~Speaker()
{
    pthread_kill(pthread_id, 15);
    free(buff);
    pthread_kill(pthread_id_ambient, 15);
    free(buff_ambient);
    snd_pcm_close(pcm_handle);
}