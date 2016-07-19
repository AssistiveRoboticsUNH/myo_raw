from logger import MyLogger

logger = MyLogger().logger

def test():
    logger.info('teset here')
    
if __name__ == '__main__':
    import rospy
    from std_msgs.msg import Int32, Empty

    rospy.init_node('build_classifier')
    rospy.Subscriber('/exercise/mode', Int32, test)

    print "Classifier launched. Listening to message..."
    logger.info('Classifier launched. Listening to message...')

    logger.info('main log')
    test()
    rospy.spin()
