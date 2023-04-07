#ifndef MANDIBLES_HPP
#define MANDIBLES_HPP

// data that affects robot movement
enum class MandibleState {
    open, closed, error
};

static std::string MandibleStateToString(MandibleState mandibleState) 
{
    switch(mandibleState) {
        case MandibleState::open:
            return "open";
            break;
        case MandibleState::closed:
            return "closed";
            break;
        default:
            return "error";
    }
}

static MandibleState stringToMandibleState(std::string mandibleState) 
{
    if(mandibleState.compare("open") == 0) {
        return MandibleState::open;
    }
    else if(mandibleState.compare("closed") == 0) {
        return MandibleState::closed;
    }
    else {
        return MandibleState::error;
    }

}


#endif