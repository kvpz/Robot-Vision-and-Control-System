#ifndef WINGS_HPP
#define WINGS_HPP

// data that affects robot movement
enum class WingState {
    open, closed, error
};

static std::string WingStateToString(WingState wingState) 
{
    switch(wingState) {
        case WingState::open:
            return "open";
            break;
        case WingState::closed:
            return "closed";
            break;
        default:
            return "error";
    }
}

static WingState stringToWingState(std::string wingState) 
{
    if(wingState.compare("open") == 0) {
        return WingState::open;
    }
    else if(wingState.compare("closed") == 0) {
        return WingState::closed;
    }
    else {
        return WingState::error;
    }

}


#endif