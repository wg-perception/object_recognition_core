//          Copyright John W. Wilkinson 2007 - 2009.
// Distributed under the MIT License, see accompanying file LICENSE.txt

// json spirit version 4.03

#include "json_spirit_writer.h"
#include "json_spirit_writer_template.h"

void or_json::write( const Value& value, std::ostream& os )
{
    write_stream( value, os, false );
}

void or_json::write_formatted( const Value& value, std::ostream& os )
{
    write_stream( value, os, true );
}

std::string or_json::write( const Value& value )
{
    return write_string( value, false );
}

std::string or_json::write_formatted( const Value& value )
{
    return write_string( value, true );
}

#ifndef BOOST_NO_STD_WSTRING

void or_json::write( const wValue& value, std::wostream& os )
{
    write_stream( value, os, false );
}

void or_json::write_formatted( const wValue& value, std::wostream& os )
{
    write_stream( value, os, true );
}

std::wstring or_json::write( const wValue&  value )
{
    return write_string( value, false );
}

std::wstring or_json::write_formatted( const wValue&  value )
{
    return write_string( value, true );
}

#endif

void or_json::write( const mValue& value, std::ostream& os )
{
    write_stream( value, os, false );
}

void or_json::write_formatted( const mValue& value, std::ostream& os )
{
    write_stream( value, os, true );
}

std::string or_json::write( const mValue& value )
{
    return write_string( value, false );
}

std::string or_json::write_formatted( const mValue& value )
{
    return write_string( value, true );
}

#ifndef BOOST_NO_STD_WSTRING

void or_json::write( const wmValue& value, std::wostream& os )
{
    write_stream( value, os, false );
}

void or_json::write_formatted( const wmValue& value, std::wostream& os )
{
    write_stream( value, os, true );
}

std::wstring or_json::write( const wmValue&  value )
{
    return write_string( value, false );
}

std::wstring or_json::write_formatted( const wmValue&  value )
{
    return write_string( value, true );
}

#endif
