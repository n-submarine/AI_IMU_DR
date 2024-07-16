import rosbag

def list_topics(bag_file):
    bag = rosbag.Bag(bag_file)
    info = bag.get_type_and_topic_info()
    bag.close()
    return info.topics

if __name__ == "__main__":
    bag_file = '/home/hsy/Bagfiles/songdo_230923.bag'
    topics = list_topics(bag_file)
    print("Available topics:")
    for topic in topics:
        print(topic)
