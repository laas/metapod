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

#ifndef METAPOD_ROBOT_BUILDER_TXTTEMPLATE_HH
# define METAPOD_ROBOT_BUILDER_TXTTEMPLATE_HH

# include <string>
# include <map>
# include <iostream>

namespace metapod {
//
// example:
//
//  TxtTemplate tt("Hello @name@! I'm feeling @mood@.");
//  std::map<std::string, std::string> repl;
//  repl["name"] = "world";
//  repl["mood"] = std::string("lucky");
//  std::cout << tt.format(repl) << std::endl;
//  tt.format(repl, std::cout);
//  std::cout << std::endl;
class TxtTemplate
{
typedef std::map<std::string, std::string> map;
public:
  TxtTemplate();
  TxtTemplate(const std::string& template_text);
  TxtTemplate(const char *template_text, size_t template_text_len);
  TxtTemplate(const char *template_text);
  TxtTemplate(const TxtTemplate&);
  ~TxtTemplate();
  void format(const map& repl, std::ostream& out) const;
  std::string format(const map& repl) const;
private:
  const std::string tmpl_;
};
}
#endif
