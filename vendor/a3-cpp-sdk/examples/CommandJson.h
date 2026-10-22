#pragma once
#include <cmath>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

// Strict, bounded command grammar: a flat object of strings, finite numbers,
// booleans and numeric arrays. Nested objects/null are not command arguments.
struct CommandJson {
    struct Value {
        enum Kind { String, Number, Boolean, Array } kind = Number;
        std::string text;
        double number = 0;
        std::vector<double> array;
    };
    std::map<std::string, Value> fields;
    static bool parse(const std::string &input, CommandJson *out) {
        struct Parser {
            const std::string &s; size_t p = 0;
            void ws() { while (p < s.size() && (s[p]==' ' || s[p]=='\t' || s[p]=='\r' || s[p]=='\n')) ++p; }
            bool take(char c) { ws(); if (p == s.size() || s[p] != c) return false; ++p; return true; }
            bool string(std::string &v) {
                if (!take('"')) return false;
                while (p < s.size()) {
                    unsigned char c = s[p++];
                    if (c == '"') return true;
                    if (c < 32) return false;
                    if (c != '\\') { v += char(c); continue; }
                    if (p == s.size()) return false;
                    c = s[p++];
                    switch(c) {
                    case '"': case '\\': case '/': v += char(c); break;
                    case 'b': v += '\b'; break; case 'f': v += '\f'; break;
                    case 'n': v += '\n'; break; case 'r': v += '\r'; break; case 't': v += '\t'; break;
                    case 'u': {
                        unsigned code = 0;
                        if (p + 4 > s.size()) return false;
                        for (int i=0;i<4;++i) {
                            char h=s[p++]; int n=h>='0'&&h<='9'?h-'0':h>='a'&&h<='f'?h-'a'+10:h>='A'&&h<='F'?h-'A'+10:-1;
                            if (n<0) return false; code = code*16+unsigned(n);
                        }
                        if (code>=0xD800 && code<=0xDFFF) return false;
                        if (code<128) v += char(code);
                        else if (code<2048) { v += char(0xC0|(code>>6)); v += char(0x80|(code&63)); }
                        else { v += char(0xE0|(code>>12)); v += char(0x80|((code>>6)&63)); v += char(0x80|(code&63)); }
                        break;
                    }
                    default: return false;
                    }
                }
                return false;
            }
            bool number(double &v) {
                ws(); size_t begin=p;
                if (p<s.size() && s[p]=='-') ++p;
                if (p==s.size()) return false;
                if (s[p]=='0') ++p;
                else {
                    if (s[p]<'1'||s[p]>'9') return false;
                    while(p<s.size() && s[p]>='0' && s[p]<='9') ++p;
                }
                if (p<s.size() && s[p]=='.') {
                    ++p; size_t digits=p;
                    while(p<s.size() && s[p]>='0' && s[p]<='9') ++p;
                    if (p==digits) return false;
                }
                if (p<s.size() && (s[p]=='e'||s[p]=='E')) {
                    ++p; if(p<s.size() && (s[p]=='+'||s[p]=='-')) ++p;
                    size_t digits=p; while(p<s.size() && s[p]>='0' && s[p]<='9') ++p;
                    if(p==digits) return false;
                }
                const auto token=s.substr(begin,p-begin); char *end=nullptr;
                v=std::strtod(token.c_str(), &end);
                return end==token.c_str()+token.size() && std::isfinite(v);
            }
            bool value(Value &v) {
                ws(); if(p==s.size()) return false;
                if(s[p]=='"') { v.kind=Value::String; return string(v.text); }
                if(s[p]=='[') {
                    ++p; v.kind=Value::Array; if(take(']')) return true;
                    do { double n; if(!number(n)) return false; v.array.push_back(n); } while(take(','));
                    return take(']');
                }
                if(s.compare(p,4,"true")==0) { p+=4; v.kind=Value::Boolean; v.number=1; return true; }
                if(s.compare(p,5,"false")==0) { p+=5; v.kind=Value::Boolean; return true; }
                return number(v.number);
            }
        } parser{input};
        CommandJson result;
        if (!parser.take('{')) return false;
        if (!parser.take('}')) {
            do {
                std::string key; Value value;
                if(!parser.string(key) || !parser.take(':') || !parser.value(value) ||
                    !result.fields.emplace(key, std::move(value)).second) return false;
            } while(parser.take(','));
            if(!parser.take('}')) return false;
        }
        parser.ws(); if(parser.p!=input.size()) return false;
        *out=std::move(result); return true;
    }
    const Value *get(const char *key) const {
        auto it=fields.find(key); return it==fields.end()?nullptr:&it->second;
    }
    bool str(const char *key, std::string *out) const {
        auto v=get(key); if(!v || v->kind!=Value::String) return false; *out=v->text; return true;
    }
    bool num(const char *key, double *out) const {
        auto v=get(key); if(!v || (v->kind!=Value::Number && v->kind!=Value::Boolean)) return false;
        *out=v->number; return true;
    }
    bool arr(const char *key, std::vector<double> *out) const {
        auto v=get(key); if(!v || v->kind!=Value::Array) return false; *out=v->array; return true;
    }
};
