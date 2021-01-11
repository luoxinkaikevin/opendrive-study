/*
 * opendrive2op_map_converter_core.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "op_planner/xml_helpers.h"
#include <fstream>

namespace opendrive
{

    // TiXmlElement对应于XML的元素
    // 递归查找和name相同的元素
    void XmlHelpers::findElements(
        std::string name, TiXmlElement *parent_element, std::vector<TiXmlElement *> &element_list)
    {
        if (parent_element == nullptr)
            return;
        // 如果name字符串与parent_element->Value()相同,则写入element_list中
        else if (name.compare(parent_element->Value()) == 0)
        {
            element_list.push_back(parent_element);
        }
        // FirstChildElement(const char* value=0):获取第一个值为value的子节点，
        // value默认值为空，则返回第一个子节点
        findElements(name, parent_element->FirstChildElement(), element_list);
        // NextSiblingElement( const char* _value=0 ) ：获得下一个(兄弟)节点。
        findElements(name, parent_element->NextSiblingElement(), element_list);
    }

    // 在parent_element中递归查找和name相同的第一个元素,只要第一个!!返回给element_list
    void XmlHelpers::findFirstElement(
        std::string name, TiXmlElement *parent_element, std::vector<TiXmlElement *> &element_list)
    {
        if (parent_element == nullptr || element_list.size() > 0)
            return;
        else if (name.compare(parent_element->Value()) == 0)
        {
            element_list.push_back(parent_element);
            return;
        }

        findFirstElement(name, parent_element->FirstChildElement(), element_list);
        findFirstElement(name, parent_element->NextSiblingElement(), element_list);
    }

    // 在parent_element中递归查找和name相同的第一个元素,只要第一个!!返回给element_list
    void XmlHelpers::findFirstElementInOneRoad(
        std::string name, TiXmlElement *parent_element, std::vector<TiXmlElement *> &element_list)
    {
        if (parent_element == nullptr || element_list.size() > 0)
            return;
        else if (name.compare(parent_element->Value()) == 0)
        {
            element_list.push_back(parent_element);
            return;
        }

        findFirstElement(name, parent_element->FirstChildElement(), element_list);
    }

    // TiXmlAttribute：对应于XML中的元素的属性。
    // 如果元素及其属性不为空,则将属性转化为int型,否则返回def_val
    int XmlHelpers::getIntAttribute(TiXmlElement *p_elem, std::string name, int def_val)
    {
        if (p_elem != nullptr && p_elem->Attribute(name) != nullptr)
            // strtol是一个C语言函数，作用就是将一个字符串转换为长整型long，其函数原型为：
            // long int strtol (const char* str, char** endptr, int base);
            return strtol(p_elem->Attribute(name.c_str()), NULL, 10);
        else
            return def_val;
    }
    // 如果元素及其属性不为空,则将属性转化为double型,否则返回def_val
    double XmlHelpers::getDoubleAttribute(TiXmlElement *p_elem, std::string name, double def_val)
    {
        if (p_elem != nullptr && p_elem->Attribute(name) != nullptr)
            return strtod(p_elem->Attribute(name.c_str()), NULL);
        else
            return def_val;
    }
    // 如果元素及其属性不为空,则将属性转化为string型,否则返回def_val
    std::string XmlHelpers::getStringAttribute(TiXmlElement *p_elem, std::string name, std::string def_val)
    {
        if (p_elem != nullptr && p_elem->Attribute(name) != nullptr)
            return std::string(p_elem->Attribute(name.c_str()));
        else
            return def_val;
    }
    // 如果元素的值不为空,则以字符串形式返回元素的值,否则返回def_val
    std::string XmlHelpers::getStringValue(TiXmlElement *p_elem, std::string def_val)
    {
        if (p_elem != nullptr && p_elem->Value() != nullptr)
            return p_elem->ValueStr();
        else
            return def_val;
    }
} // namespace opendrive
