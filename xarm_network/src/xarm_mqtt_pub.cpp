#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_msgs.h>
#include <mqtt/async_client.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <map>
#include <vector>
#include <cstring>

const std::string DFLT_SERVER_ADDRESS	{ "101.101.218.67:1883" };
const std::string CLIENT_ID				{ "og_publish" };
const std::string PERSIST_DIR			{ "./persist" };

const std::string TOPIC 				{ "real_time" };
const std::string TOPIC_VOLATAGE        { "real_time/voltage" };
const std::string TOPIC_CURRENT         { "real_time/current" };

const char* LWT_PAYLOAD = "Last will and testament.";

const int  QOS = 1;

const auto TIMEOUT = std::chrono::seconds(10);

int state_tmp = -1;
int err_tmp = -1;

class callback : public virtual mqtt::callback
{
public:
	void connection_lost(const std::string& cause) override {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;
	}

	void delivery_complete(mqtt::delivery_token_ptr tok) override {
		std::cout << "\tDelivery complete for token: "
			<< (tok ? tok->get_message_id() : -1) << std::endl;
	}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * A base action listener.
 */
class action_listener : public virtual mqtt::iaction_listener
{
protected:
	void on_failure(const mqtt::token& tok) override {
		std::cout << "\tListener failure for token: "
			<< tok.get_message_id() << std::endl;
	}

	void on_success(const mqtt::token& tok) override {
		std::cout << "\tListener success for token: "
			<< tok.get_message_id() << std::endl;
	}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * A derived action listener for publish events.
 */
class delivery_action_listener : public action_listener
{
	std::atomic<bool> done_;

	void on_failure(const mqtt::token& tok) override {
		action_listener::on_failure(tok);
		done_ = true;
	}

	void on_success(const mqtt::token& tok) override {
		action_listener::on_success(tok);
		done_ = true;
	}

public:
	delivery_action_listener() : done_(false) {}
	bool is_done() const { return done_; }
};

bool mqtt_pub_robot_data(const xarm_msgs::msg::RobotMsg::SharedPtr states){
	rapidjson::Document d;
	rapidjson::Value sn;
	
	char sn_buffer[states->serial_number.size()];
	
	int sn_len = sprintf(sn_buffer, "%s", states->serial_number.c_str());

	sn.SetString(sn_buffer, sn_len, d.GetAllocator());

    rapidjson::Value robot_voltage(rapidjson::kObjectType);
    rapidjson::Value robot_voltage_data(rapidjson::kObjectType);

	sn_len = sprintf(sn_buffer, "%s", states->serial_number.c_str());

	sn.SetString(sn_buffer, sn_len, d.GetAllocator());

    robot_voltage.AddMember("robot_serial", sn, d.GetAllocator());

	rapidjson::Value vol_tmp;
    for(int i = 0; i < 6; i++){
        std::string key_v_str = "robot_current";
    	key_v_str += std::to_string(i+1);
    	rapidjson::Value key_v(key_v_str.c_str(), d.GetAllocator());
		vol_tmp.SetFloat(states->voltages[i]);
		robot_voltage_data.AddMember(key_v, vol_tmp, d.GetAllocator());
    }

    robot_voltage.AddMember("robot_data", robot_voltage_data, d.GetAllocator());

    rapidjson::StringBuffer sb_vol;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb_vol);
	robot_voltage.Accept(writer);
	std::cout << sb_vol.GetString() << std::endl;

    rapidjson::Value robot_current(rapidjson::kObjectType);
    rapidjson::Value robot_current_data(rapidjson::kObjectType);

	sn_len = sprintf(sn_buffer, "%s", states->serial_number.c_str());

	sn.SetString(sn_buffer, sn_len, d.GetAllocator());

    robot_current.AddMember("robot_serial", sn, d.GetAllocator());

	rapidjson::Value cur_tmp;
    for(int i = 0; i < 6; i++){
        std::string key_c_str = "robot_voltage";
        key_c_str += std::to_string(i+1);
		rapidjson::Value key_c(key_c_str.c_str(), d.GetAllocator());
		cur_tmp.SetFloat(states->currents[i]);
		robot_current_data.AddMember(key_c, cur_tmp, d.GetAllocator());
    }

    robot_current.AddMember("robot_data", robot_current_data, d.GetAllocator());

    rapidjson::StringBuffer sb_cur;
	writer.Reset(sb_cur);
	robot_current.Accept(writer);
	std::cout << sb_cur.GetString() << std::endl;

	std::string	address  = DFLT_SERVER_ADDRESS;
	std::string clientID = CLIENT_ID;

	std::cout << "Initializing for server '" << address << "'..." << std::endl;
	mqtt::async_client client(address, clientID, PERSIST_DIR);

	callback cb;
	client.set_callback(cb);

	auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();

	std::cout << "  ...OK" << std::endl;

	try {
		std::cout << "\nConnecting..." << std::endl;
		mqtt::token_ptr conntok = client.connect(connOpts);
		std::cout << "Waiting for the connection..." << std::endl;
		conntok->wait();
		std::cout << "  ...OK" << std::endl;

		mqtt::message_ptr pub_voltage_msg = mqtt::make_message(TOPIC_VOLATAGE, sb_vol.GetString());
        pub_voltage_msg->set_qos(QOS);
        client.publish(pub_voltage_msg)->wait_for(TIMEOUT);

        mqtt::message_ptr pub_current_msg = mqtt::make_message(TOPIC_CURRENT, sb_cur.GetString());
        pub_current_msg->set_qos(QOS);
        client.publish(pub_current_msg)->wait_for(TIMEOUT);

		// Disconnect
		std::cout << "\nDisconnecting..." << std::endl;
		client.disconnect()->wait();
		std::cout << "  ...OK" << std::endl;
	}
	catch (const mqtt::exception& exc) {
		std::cerr << exc.what() << std::endl;
		return 1;
	}
	return 0;
}

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_mqtt_pub] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    std::string hw_ns = "xarm";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_mqtt_pub", hw_ns);
    RCLCPP_INFO(node->get_logger(), "xarm_mqtt_pub start");

	rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub = node->create_subscription<xarm_msgs::msg::RobotMsg>("xarm_states", 100, mqtt_pub_robot_data);

    signal(SIGINT, exit_sig_handler);
	rclcpp::spin(node);
    rclcpp::shutdown();

	RCLCPP_INFO(node->get_logger(), "Exiting");
 	return 0;
}