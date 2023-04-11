#ifndef ATTRACTIONCOLORS_HPP
#define ATTRACTIONCOLORS_HPP

enum class AttractionColors : char {
    GREEN, RED, NONE
};

static std::string AttractionColorsToString(AttractionColors state) 
{
    switch(state) {
        case AttractionColors::GREEN:
            return "GREEN";
        case AttractionColors::RED:
            return "RED";
        case AttractionColors::NONE:
            return "NONE";
    }
}

static AttractionColors StringToAttractionColor(std::string attractionColor) 
{
    if(attractionColor == "RED")
        return AttractionColors::RED;
    else if(attractionColor == "GREEN")
        return AttractionColors::GREEN;
    else
        return AttractionColors::NONE;
}

#endif