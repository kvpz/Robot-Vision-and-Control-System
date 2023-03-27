#ifndef TRAVELDIRECTION_HPP
#define TRAVELDIRECTION_HPP

// data that affects robot movement
enum class TravelDirection {
    forward, backward, leftward, rightward, error
};

static std::string travelDirectionToString(TravelDirection travelDirection) 
{
    switch(travelDirection) {
        case TravelDirection::forward:
            return "forward";
            break;
        case TravelDirection::backward:
            return "backward";
            break;
        case TravelDirection::leftward:
            return "leftward";
            break;
        case TravelDirection::rightward:
            return "rightward";
            break;
        default:
            return "error";
    }
}

static TravelDirection stringToTravelDirection(std::string travelDirection) 
{
    if(travelDirection.compare("forward") == 0) {
        return TravelDirection::forward;
    }
    else if(travelDirection.compare("backward") == 0) {
        return TravelDirection::backward;
    }
    else if(travelDirection.compare("rightward") == 0) {
        return TravelDirection::rightward;
    }
    else if(travelDirection.compare("leftward") == 0) {
        return TravelDirection::leftward;
    }
    else {
        return TravelDirection::error;
    }

}


#endif