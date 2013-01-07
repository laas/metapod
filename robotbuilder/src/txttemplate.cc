// Copyright 2013
//
// Sébastien Barthélémy (Aldebaran Robotics)
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

#include "txttemplate.hh"
#include <boost/regex.hpp>
#include <boost/algorithm/string/find_format.hpp>
#include <boost/algorithm/string/regex_find_format.hpp>

namespace {
// Functor that provides the replacement text given a match for use with
// boost::algorithm::find_format_all_copy or boost::regex_replace
// (only possible with boost >= 1.42).
//
// Match/replacement pairs are stored in a map.
//
// Boost::regex_replace passes the functor by copy. To avoid copying map, we
// only store a reference.
class ReplFunctor
{
private:
  const std::map<std::string, std::string> & map_;
public:
  ReplFunctor(const std::map<std::string, std::string> & map):
      map_(map)
  {}
  // version for boost::regex_replace
  template <typename IteratorT>
  std::string operator()(
      boost::match_results<IteratorT> m) const
  {
    std::string key = m[1].str(); // will this compile for any IteratorT?
    std::map<std::string, std::string>::const_iterator it = map_.find(key);
    if (it == map_.end())
      return std::string();
    else
      return it->second;
  }
  // version for boost::algorithm::find_format_all_copy
  template <typename IteratorT>
  std::string operator()(
      boost::algorithm::detail::regex_search_result<IteratorT> m) const
  {
    return operator()(m.match_results());
  }
};

}

namespace metapod {

TxtTemplate::TxtTemplate()
{
}

TxtTemplate::TxtTemplate(const std::string& template_text):
  tmpl_(template_text)
{}

TxtTemplate::TxtTemplate(const char *template_text, size_t template_text_len):
  tmpl_(template_text, template_text_len)
{}

TxtTemplate::TxtTemplate(const char *template_text):
  tmpl_(template_text)
{}

TxtTemplate::~TxtTemplate()
{}

std::string TxtTemplate::format(const map& repl) const
{
  boost::regex e("@(([a-zA-Z_][a-zA-Z0-9_]+))@");
  ReplFunctor formatter(repl);
  return boost::algorithm::find_format_all_copy(
      tmpl_,
      boost::algorithm::regex_finder(e, boost::regex_constants::format_literal),
      formatter);
}

void TxtTemplate::format(const map& repl, std::ostream& os) const
{
  os << format(repl);
}

}
