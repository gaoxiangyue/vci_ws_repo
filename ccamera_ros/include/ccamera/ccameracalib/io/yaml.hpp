#ifndef CE_ARCIVES_YAML_HPP_
#define CE_ARCIVES_YAML_HPP_

#include <limits>
#include <sstream>
#include <stack>
#include <vector>
#include <string>

#include <cereal/cereal.hpp>
#include <cereal/details/util.hpp>
#include <cereal/external/base64.hpp>
#include <cereal/archives/json.hpp>

#include <yaml-cpp/yaml.h>

#ifndef CEREAL_NOEXCEPT
  #ifdef CEREAL_HAS_NOEXCEPT
    #define CEREAL_NOEXCEPT noexcept
  #else
    #define CEREAL_NOEXCEPT
  #endif // end CEREAL_HAS_NOEXCEPT
#endif // end !defined(CEREAL_HAS_NOEXCEPT)

using namespace cereal;

const std::string empty = std::string();

namespace ce
{

  // ######################################################################
  //! An output archive designed to save data to YAML
  class YAMLOutputArchive : public OutputArchive<YAMLOutputArchive>, public traits::TextArchive
  {
    enum class NodeType { StartObject, InObject, StartArray, InArray };

    public:
      /*! @name Common Functionality
          Common use cases for directly interacting with an YAMLOutputArchive */
      //! @{

      //! A class containing various advanced options for the YAML archive
      class Options
      {
        public:
          //! Default options
          static Options Default(){ return Options(); }

          //! Default options with no indentation
          static Options NoIndent(){ return Options(); }

          //! Specify specific options for the YAMLOutputArchive
          explicit Options( ) {}
      };

      //! Construct, outputting to the provided stream
      /*! @param stream The stream to output to.
          @param options The YAML specific options to use.  See the Options struct
                         for the values of default parameters */
      YAMLOutputArchive(std::ostream & stream, Options const & options = Options::Default() ) :
        OutputArchive<YAMLOutputArchive>(this),
        itsWriteStream(stream),
        itsWriter(),
        itsNextName(),
        itsNextTag()
      {
        itsWriter.SetSeqFormat(YAML::EMITTER_MANIP::Flow);
        itsNameCounter.push(0);
        itsNodeStack.push(NodeType::StartObject);
      }

      //! Destructor, flushes the YAML
      ~YAMLOutputArchive() CEREAL_NOEXCEPT
      {
        if (itsNodeStack.top() == NodeType::InObject)
          itsWriter << YAML::EndSeq;
        else if (itsNodeStack.top() == NodeType::InArray)
          itsWriter << YAML::EndMap;

        itsWriteStream << "%YAML:1.0" << std::endl;
        itsWriteStream << itsWriter.c_str();

        itsWriteStream.flush();
      }

      //! Saves some binary data, encoded as a base64 string, with an optional name
      /*! This will create a new node, optionally named, and insert a value that consists of
          the data encoded as a base64 string */
      void saveBinaryValue( const void * data, size_t size, std::string const & name = empty )
      {
        setNextName( name );
        writeName();

        auto base64string = base64::encode( reinterpret_cast<const unsigned char *>( data ), size );
        saveValue( base64string );
      }

      //! @}
      /*! @name Internal Functionality
          Functionality designed for use by those requiring control over the inner mechanisms of
          the YAMLOutputArchive */
      //! @{

      //! Starts a new node in the YAML output
      /*! The node can optionally be given a name by calling setNextName prior
          to creating the node

          Nodes only need to be started for types that are themselves objects or arrays */
      void startNode()
      {
        writeName();
        itsNodeStack.push(NodeType::StartObject);
        itsNameCounter.push(0);
      }

      //! Designates the most recently added node as finished
      void finishNode()
      {
        // if we ended up serializing an empty object or array, writeName
        // will never have been called - so start and then immediately end
        // the object/array.
        //
        // We'll also end any object/arrays we happen to be in
        switch(itsNodeStack.top())
        {
          case NodeType::StartArray:
            itsWriter << YAML::BeginSeq;
          case NodeType::InArray:
            itsWriter << YAML::EndSeq;
            break;
          case NodeType::StartObject:
            itsWriter << YAML::BeginMap;
          case NodeType::InObject:
            itsWriter << YAML::EndMap;
            break;
        }

        itsNodeStack.pop();
        itsNameCounter.pop();
      }

      //! Sets the name for the next node created with startNode
      void setNextName( std::string const & name )
      {
        itsNextName = name;
      }

      //! Saves a bool to the current node
      void saveValue(bool b)                { itsWriter << YAML::Value << b; }
      //! Saves an int to the current node
      void saveValue(int i)                 { itsWriter << YAML::Value << i; }
      //! Saves a uint to the current node
      void saveValue(unsigned u)            { itsWriter << YAML::Value << u; }
      //! Saves an int64 to the current node
      void saveValue(int64_t i64)           { itsWriter << YAML::Value << i64; }
      //! Saves a uint64 to the current node
      void saveValue(uint64_t u64)          { itsWriter << YAML::Value << u64; }
      //! Saves a double to the current node
      void saveValue(double d)              { itsWriter << YAML::Value << d; }
      //! Saves a string to the current node
      void saveValue(std::string const & s) {
          if (doubleQuoted)
          {
            itsWriter << YAML::DoubleQuoted;
            itsWriter << YAML::Value << s;
            itsWriter << YAML::Auto;
          }
          else
            itsWriter << YAML::Value << s;
      }
      //! Saves a const char * to the current node
      void saveValue(char const * s)        { itsWriter << YAML::Value << s; }
      //! Saves a nullptr to the current node
      void saveValue(std::nullptr_t)        { itsWriter << YAML::Value << YAML::Null; } //or ""?

      bool doubleQuoted = false;
      void putStringsQuoted() { doubleQuoted = true; }
      void putStringsPlain() { doubleQuoted = false; }

    private:
      // Some compilers/OS have difficulty disambiguating the above for various flavors of longs, so we provide
      // special overloads to handle these cases.

      //! 32 bit signed long saving to current node
      template <class T, traits::EnableIf<sizeof(T) == sizeof(std::int32_t),
                                          std::is_signed<T>::value> = traits::sfinae> inline
      void saveLong(T l){ saveValue( static_cast<std::int32_t>( l ) ); }

      //! non 32 bit signed long saving to current node
      template <class T, traits::EnableIf<sizeof(T) != sizeof(std::int32_t),
                                          std::is_signed<T>::value> = traits::sfinae> inline
      void saveLong(T l){ saveValue( static_cast<std::int64_t>( l ) ); }

      //! 32 bit unsigned long saving to current node
      template <class T, traits::EnableIf<sizeof(T) == sizeof(std::int32_t),
                                          std::is_unsigned<T>::value> = traits::sfinae> inline
      void saveLong(T lu){ saveValue( static_cast<std::uint32_t>( lu ) ); }

      //! non 32 bit unsigned long saving to current node
      template <class T, traits::EnableIf<sizeof(T) != sizeof(std::int32_t),
                                          std::is_unsigned<T>::value> = traits::sfinae> inline
      void saveLong(T lu){ saveValue( static_cast<std::uint64_t>( lu ) ); }

    public:
#ifdef _MSC_VER
      //! MSVC only long overload to current node
      void saveValue( unsigned long lu ){ saveLong( lu ); };
#else // _MSC_VER
      //! Serialize a long if it would not be caught otherwise
      template <class T, traits::EnableIf<std::is_same<T, long>::value,
                                          !std::is_same<T, std::int32_t>::value,
                                          !std::is_same<T, std::int64_t>::value> = traits::sfinae> inline
      void saveValue( T t ){ saveLong( t ); }

      //! Serialize an unsigned long if it would not be caught otherwise
      template <class T, traits::EnableIf<std::is_same<T, unsigned long>::value,
                                          !std::is_same<T, std::uint32_t>::value,
                                          !std::is_same<T, std::uint64_t>::value> = traits::sfinae> inline
      void saveValue( T t ){ saveLong( t ); }
#endif // _MSC_VER

      //! Save exotic arithmetic as strings to current node
      /*! Handles long long (if distinct from other types), unsigned long (if distinct), and long double */
      template <class T, traits::EnableIf<std::is_arithmetic<T>::value,
                                          !std::is_same<T, long>::value,
                                          !std::is_same<T, unsigned long>::value,
                                          !std::is_same<T, std::int64_t>::value,
                                          !std::is_same<T, std::uint64_t>::value,
                                          (sizeof(T) >= sizeof(long double) || sizeof(T) >= sizeof(long long))> = traits::sfinae> inline
      void saveValue(T const & t)
      {
        std::stringstream ss;
        ss.precision( std::numeric_limits<long double>::max_digits10 );
        ss << t;
        saveValue( ss.str() );
      }

      //! Write the name of the upcoming node and prepare object/array state
      /*! Since writeName is called for every value that is output, regardless of
          whether it has a name or not, it is the place where we will do a deferred
          check of our node state and decide whether we are in an array or an object.

          The general workflow of saving to the YAML archive is:

            1. (optional) Set the name for the next node to be created, usually done by an NVP
            2. Start the node
            3. (if there is data to save) Write the name of the node (this function)
            4. (if there is data to save) Save the data (with saveValue)
            5. Finish the node
          */
      void writeName()
      {
        if(!itsNextTag.empty()) //now used for opencv mat tagging back compatibility
          writeTag();

        NodeType const & nodeType = itsNodeStack.top();

        if(nodeType == NodeType::StartArray)
        {
          itsWriter << YAML::BeginSeq;
          itsNodeStack.top() = NodeType::InArray;
        }
        else if(nodeType == NodeType::StartObject)
        {
          itsNodeStack.top() = NodeType::InObject;
          itsWriter << YAML::BeginMap;
        }

        // Array types do not output names
        if(nodeType == NodeType::InArray) return;

        // Start up either an object or an array, depending on state
        itsWriter << YAML::Key;

        if(itsNextName.empty())
        {
          std::string name = "value" + std::to_string( itsNameCounter.top()++ );
          saveValue(name);
        }
        else
        {
          saveValue(itsNextName);
          itsNextName = empty;
        }


      }

      void writeTag()
      {
        itsWriter << YAML::_Tag("", itsNextTag, YAML::_Tag::Type::NamedHandle);
        itsNextTag = empty;
      }

      void setNextTag( std::string tag )
      {
          itsNextTag = tag;
      }

      //! Designates that the current node should be output as an array, not an object
      void makeArray()
      {
        itsNodeStack.top() = NodeType::StartArray;
      }

      //! @}

    private:
      std::ostream  &itsWriteStream;
      YAML::Emitter itsWriter;
      std::string itsNextName;            //!< The next name
      std::string itsNextTag;             //!< Tag
      std::stack<uint32_t> itsNameCounter; //!< Counter for creating unique names for unnamed nodes
      std::stack<NodeType> itsNodeStack;
  }; // YAMLOutputArchive


  // ######################################################################
  //! An input archive designed to load data from YAML
  /*! This archive uses RapidYAML to read in a YAML archive.

      As with the output YAML archive, the preferred way to use this archive is in
      an RAII fashion, ensuring its destruction after all data has been read.

      Input YAML should have been produced by the YAMLOutputArchive.  Data can
      only be added to dynamically sized containers (marked by YAML arrays) -
      the input archive will determine their size by looking at the number of child nodes.
      Only YAML originating from a YAMLOutputArchive is officially supported, but data
      from other sources may work if properly formatted.

      The YAMLInputArchive does not require that nodes are loaded in the same
      order they were saved by YAMLOutputArchive.  Using name value pairs (NVPs),
      it is possible to load in an out of order fashion or otherwise skip/select
      specific nodes to load.

      The default behavior of the input archive is to read sequentially starting
      with the first node and exploring its children.  When a given NVP does
      not match the read in name for a node, the archive will search for that
      node at the current level and load it if it exists.  After loading an out of
      order node, the archive will then proceed back to loading sequentially from
      its new position.

      Consider this simple example where loading of some data is skipped:

      @code{cpp}
      // imagine the input file has someData(1-9) saved in order at the top level node
      ar( someData1, someData2, someData3 );        // XML loads in the order it sees in the file
      ar( cereal::make_nvp( "hello", someData6 ) ); // NVP given does not
                                                    // match expected NVP name, so we search
                                                    // for the given NVP and load that value
      ar( someData7, someData8, someData9 );        // with no NVP given, loading resumes at its
                                                    // current location, proceeding sequentially
      @endcode

      \ingroup Archives */
  class YAMLInputArchive : public InputArchive<YAMLInputArchive>, public traits::TextArchive
  {
    public:
      /*! @name Common Functionality
          Common use cases for directly interacting with an YAMLInputArchive */
      //! @{

      //! Construct, reading from the provided stream
      /*! @param stream The stream to read from */
      YAMLInputArchive(std::istream & stream) :
        InputArchive<YAMLInputArchive>(this),
        itsNextName(),
        itsReadStream(stream)
      {
        itsDocument = YAML::Load(itsReadStream);

        itsIteratorStack.emplace_back(itsDocument.begin(), itsDocument.end(), IteratorType::Member);
      }

      ~YAMLInputArchive() CEREAL_NOEXCEPT = default;

      //! Loads some binary data, encoded as a base64 string
      /*! This will automatically start and finish a node to load the data, and can be called directly by
          users.

          Note that this follows the same ordering rules specified in the class description in regards
          to loading in/out of order */
      void loadBinaryValue( void * data, size_t size, std::string const & name = empty )
      {
        itsNextName = name;

        std::string encoded;
        loadValue( encoded );
        auto decoded = base64::decode( encoded );

        if( size != decoded.size() )
          throw Exception("Decoded binary data size does not match specified size");

        std::memcpy( data, decoded.data(), decoded.size() );
        itsNextName = empty;
      }

    private:
      //! @}
      /*! @name Internal Functionality
          Functionality designed for use by those requiring control over the inner mechanisms of
          the YAMLInputArchive */
      //! @{

      //! An internal iterator that handles both array and object types
      /*! This class is a variant and holds both types of iterators that
          rapidYAML supports - one for arrays and one for objects. */
      enum IteratorType {Value, Member, Null_};    //!< Whether this holds values (array) or members (objects) or nothing

      class Iterator
      {
        public:
          Iterator() : itsIndex( 0 ), itsType(Null_) {}

          Iterator(YAML::Node::iterator begin, YAML::Node::iterator end, IteratorType type) :
            itsMemberItBegin(begin), itsMemberItEnd(end), itsIndex(0), itsType(type)
          {
            itsMemberItCurrent = itsMemberItBegin;
          }

          //! Advance to the next node
          Iterator & operator++()
          {
            ++itsIndex;
            ++itsMemberItCurrent;
            return *this;
          }

          //! Get the value of the current node
          template<typename T>
          T value()
          {
            switch(itsType)
            {
              case Value : return itsMemberItCurrent->as<T>();
              case Member: return itsMemberItCurrent->second.as<T>();
              default: throw cereal::Exception("Invalid Iterator Type!");
            }
          }

          YAML::Node::iterator &node() {
              return itsMemberItCurrent;
          }


          //! Get the name of the current node, or nullptr if it has no name
          std::string name() const
          {
            if( itsType == Member && itsMemberItCurrent != itsMemberItEnd ) {
              return itsMemberItCurrent->first.as<std::string>();
            }
            else
              return empty;
          }

          //! Adjust our position such that we are at the node with the given name
          /*! @throws Exception if no such named node exists */
          inline void search( std::string const & searchName )
          {
            const auto len = searchName.length();
            size_t index = 0;
            for( auto it = itsMemberItBegin; it != itsMemberItEnd; ++it, ++index )
            {
              auto const & currentName = it->first.as<std::string>();
              if( ( searchName.compare(currentName) == 0 ) &&
                  ( currentName.length() == len ) )
              {
                itsIndex = index;
                itsMemberItCurrent = it;

                return;
              }
            }

            throw Exception("YAML Parsing failed - provided NVP (" + std::string(searchName) + ") not found");
          }

        private:
          YAML::Node::iterator itsMemberItBegin, itsMemberItEnd, itsMemberItCurrent; //!< The member iterator (object)
          size_t itsIndex;                                 //!< The current index of this iterator
          IteratorType itsType;
      };

      //! @}
      /*! @name Internal Functionality
          Functionality designed for use by those requiring control over the inner mechanisms of
          the YAMLInputArchive */
      //! @{


      //! Searches for the expectedName node if it doesn't match the actualName
      /*! This needs to be called before every load or node start occurs.  This function will
          check to see if an NVP has been provided (with setNextName) and if so, see if that name matches the actual
          next name given.  If the names do not match, it will search in the current level of the YAML for that name.
          If the name is not found, an exception will be thrown.

          Resets the NVP name after called.

          @throws Exception if an expectedName is given and not found */
      inline void search()
      {
          // The name an NVP provided with setNextName()
          if( !itsNextName.empty() )
          {
            // The actual name of the current node
            auto const & actualName = itsIteratorStack.back().name();

            // Do a search if we don't see a name coming up, or if the names don't match
            if( actualName.empty() || itsNextName.compare( actualName ) )
              itsIteratorStack.back().search( itsNextName );
          }

          itsNextName = empty;
      }

    public:
      //! Starts a new node, going into its proper iterator
      /*! This places an iterator for the next node to be parsed onto the iterator stack.  If the next
          node is an array, this will be a value iterator, otherwise it will be a member iterator.

          By default our strategy is to start with the document root node and then recursively iterate through
          all children in the order they show up in the document.
          We don't need to know NVPs to do this; we'll just blindly load in the order things appear in.

          If we were given an NVP, we will search for it if it does not match our the name of the next node
          that would normally be loaded.  This functionality is provided by search(). */
      void startNode()
      {
        search();

        if(itsIteratorStack.back().node()->second.IsSequence()) {
          assert(itsIteratorStack.back().node()->second.IsSequence());
          itsIteratorStack.emplace_back(itsIteratorStack.back().node()->second.begin(), itsIteratorStack.back().node()->second.end(), IteratorType::Value);
        }
        else
          itsIteratorStack.emplace_back(itsIteratorStack.back().node()->second.begin(), itsIteratorStack.back().node()->second.end(), IteratorType::Member);
      }

      //! Finishes the most recently started node
      void finishNode()
      {
        itsIteratorStack.pop_back();
        ++itsIteratorStack.back();
      }

      //! Retrieves the current node name
      /*! @return nullptr if no name exists */
      std::string getNodeName() const
      {
        return itsIteratorStack.back().name();
      }

      //! Sets the name for the next node created with startNode
      void setNextName( std::string const & name )
      {
        itsNextName = name;
      }

      //! Loads a value from the current node - small signed overload
      template <class T, traits::EnableIf<std::is_signed<T>::value,
                                          sizeof(T) < sizeof(int64_t)> = traits::sfinae> inline
      void loadValue(T & val)
      {
        search();

        val = static_cast<T>( itsIteratorStack.back().value<int>() );
        ++itsIteratorStack.back();
      }

      //! Loads a value from the current node - small unsigned overload
      template <class T, traits::EnableIf<std::is_unsigned<T>::value,
                                          sizeof(T) < sizeof(uint64_t),
                                          !std::is_same<bool, T>::value> = traits::sfinae> inline
      void loadValue(T & val)
      {
        search();

        val = static_cast<T>( itsIteratorStack.back().value<unsigned>() );
        ++itsIteratorStack.back();
      }

      //! Loads a value from the current node - bool overload
      void loadValue(bool & val)        { search(); val = itsIteratorStack.back().value<bool>(); ++itsIteratorStack.back(); }
      //! Loads a value from the current node - int64 overload
      void loadValue(int64_t & val)     { search(); val = itsIteratorStack.back().value<int64_t>(); ++itsIteratorStack.back(); }
      //! Loads a value from the current node - uint64 overload
      void loadValue(uint64_t & val)    { search(); val = itsIteratorStack.back().value<uint64_t>(); ++itsIteratorStack.back(); }
      //! Loads a value from the current node - float overload
      void loadValue(float & val)       { search(); val = itsIteratorStack.back().value<float>(); ++itsIteratorStack.back(); }
      //! Loads a value from the current node - double overload
      void loadValue(double & val)      { search(); val = itsIteratorStack.back().value<double>(); ++itsIteratorStack.back(); }
      //! Loads a value from the current node - string overload
      void loadValue(std::string & val) { search(); val = itsIteratorStack.back().value<std::string>(); ++itsIteratorStack.back(); }
      //! Loads a nullptr from the current node
      void loadValue(std::nullptr_t&)   { search(); std::cerr << "TODO:" << std::endl; ++itsIteratorStack.back(); }

      // Special cases to handle various flavors of long, which tend to conflict with
      // the int32_t or int64_t on various compiler/OS combinations.  MSVC doesn't need any of this.
      #ifndef _MSC_VER
    private:
      //! 32 bit signed long loading from current node
      template <class T> inline
      typename std::enable_if<sizeof(T) == sizeof(std::int32_t) && std::is_signed<T>::value, void>::type
      loadLong(T & l){ loadValue( reinterpret_cast<std::int32_t&>( l ) ); }

      //! non 32 bit signed long loading from current node
      template <class T> inline
      typename std::enable_if<sizeof(T) == sizeof(std::int64_t) && std::is_signed<T>::value, void>::type
      loadLong(T & l){ loadValue( reinterpret_cast<std::int64_t&>( l ) ); }

      //! 32 bit unsigned long loading from current node
      template <class T> inline
      typename std::enable_if<sizeof(T) == sizeof(std::uint32_t) && !std::is_signed<T>::value, void>::type
      loadLong(T & lu){ loadValue( reinterpret_cast<std::uint32_t&>( lu ) ); }

      //! non 32 bit unsigned long loading from current node
      template <class T> inline
      typename std::enable_if<sizeof(T) == sizeof(std::uint64_t) && !std::is_signed<T>::value, void>::type
      loadLong(T & lu){ loadValue( reinterpret_cast<std::uint64_t&>( lu ) ); }

    public:
      //! Serialize a long if it would not be caught otherwise
      template <class T> inline
      typename std::enable_if<std::is_same<T, long>::value &&
                              sizeof(T) >= sizeof(std::int64_t) &&
                              !std::is_same<T, std::int64_t>::value, void>::type
      loadValue( T & t ){ loadLong(t); }

      //! Serialize an unsigned long if it would not be caught otherwise
      template <class T> inline
      typename std::enable_if<std::is_same<T, unsigned long>::value &&
                              sizeof(T) >= sizeof(std::uint64_t) &&
                              !std::is_same<T, std::uint64_t>::value, void>::type
      loadValue( T & t ){ loadLong(t); }
      #endif // _MSC_VER

    private:
      //! Convert a string to a long long
      void stringToNumber( std::string const & str, long long & val ) { val = std::stoll( str ); }
      //! Convert a string to an unsigned long long
      void stringToNumber( std::string const & str, unsigned long long & val ) { val = std::stoull( str ); }
      //! Convert a string to a long double
      void stringToNumber( std::string const & str, long double & val ) { val = std::stold( str ); }

    public:
      //! Loads a value from the current node - long double and long long overloads
      template <class T, traits::EnableIf<std::is_arithmetic<T>::value,
                                          !std::is_same<T, long>::value,
                                          !std::is_same<T, unsigned long>::value,
                                          !std::is_same<T, std::int64_t>::value,
                                          !std::is_same<T, std::uint64_t>::value,
                                          (sizeof(T) >= sizeof(long double) || sizeof(T) >= sizeof(long long))> = traits::sfinae>
      inline void loadValue(T & val)
      {
        std::string encoded;
        loadValue( encoded );
        stringToNumber( encoded, val );
      }

      //! Loads the size for a SizeTag
      void loadSize(size_type & size)
      {
        if (itsIteratorStack.size() == 1)
          size = itsDocument.size();
        else
          size = itsIteratorStack.rbegin()[1].node()->second.size();
      }

      //! @}

    private:
      std::string itsNextName;               //!< Next name set by NVP
      std::istream &itsReadStream;               //!< RapidYAML write stream
      std::vector<Iterator> itsIteratorStack; //!< 'Stack' of rapidYAML iterators
      YAML::Node itsDocument;                 //!< YAML tree
  };


  // ######################################################################
  // YAMLArchive prologue and epilogue functions
  // ######################################################################

  // ######################################################################
  //! Prologue for NVPs for YAML archives
  /*! NVPs do not start or finish nodes - they just set up the names */
  template <class T> inline
  void prologue( YAMLOutputArchive &, NameValuePair<T> const & )
  {  }

  //! Prologue for NVPs for YAML archives
  template <class T> inline
  void prologue( YAMLInputArchive &, NameValuePair<T> const & )
  { }

  // ######################################################################
  //! Epilogue for NVPs for YAML archives
  /*! NVPs do not start or finish nodes - they just set up the names */
  template <class T> inline
  void epilogue( YAMLOutputArchive &, NameValuePair<T> const & )
  { }

  //! Epilogue for NVPs for YAML archives
  /*! NVPs do not start or finish nodes - they just set up the names */
  template <class T> inline
  void epilogue( YAMLInputArchive &, NameValuePair<T> const & )
  { }

  // ######################################################################
  //! Prologue for SizeTags for YAML archives
  /*! SizeTags are strictly ignored for YAML, they just indicate
      that the current node should be made into an array */
  template <class T> inline
  void prologue( YAMLOutputArchive & ar, SizeTag<T> const & )
  {
    ar.makeArray();
  }

  //! Prologue for SizeTags for YAML archives
  template <class T> inline
  void prologue( YAMLInputArchive &, SizeTag<T> const & )
  { }


  // ######################################################################
  //! Epilogue for SizeTags for YAML archives
  /*! SizeTags are strictly ignored for YAML */
  template <class T> inline
  void epilogue( YAMLOutputArchive &, SizeTag<T> const & )
  { }

  //! Epilogue for SizeTags for YAML archives
  template <class T> inline
  void epilogue( YAMLInputArchive &, SizeTag<T> const & )
  { }

  // ######################################################################
  //! Prologue for all other types for YAML archives (except minimal types)
  /*! Starts a new node, named either automatically or by some NVP,
      that may be given data by the type about to be archived

      Minimal types do not start or finish nodes */
  template <class T, traits::EnableIf<!std::is_arithmetic<T>::value,
                                      !traits::has_minimal_base_class_serialization<T, traits::has_minimal_output_serialization, YAMLOutputArchive>::value,
                                      !traits::has_minimal_output_serialization<T, YAMLOutputArchive>::value> = traits::sfinae>
  inline void prologue( YAMLOutputArchive & ar, T const & )
  {
    ar.startNode();
  }

  //! Prologue for all other types for YAML archives
  template <class T, traits::EnableIf<!std::is_arithmetic<T>::value,
                                      !traits::has_minimal_base_class_serialization<T, traits::has_minimal_input_serialization, YAMLInputArchive>::value,
                                      !traits::has_minimal_input_serialization<T, YAMLInputArchive>::value> = traits::sfinae>
  inline void prologue( YAMLInputArchive & ar, T const & )
  {
    ar.startNode();
  }

  // ######################################################################
  //! Epilogue for all other types other for YAML archives (except minimal types)
  /*! Finishes the node created in the prologue

      Minimal types do not start or finish nodes */
  template <class T, traits::EnableIf<!std::is_arithmetic<T>::value,
                                      !traits::has_minimal_base_class_serialization<T, traits::has_minimal_output_serialization, YAMLOutputArchive>::value,
                                      !traits::has_minimal_output_serialization<T, YAMLOutputArchive>::value> = traits::sfinae>
  inline void epilogue( YAMLOutputArchive & ar, T const & )
  {
    ar.finishNode();
  }

  //! Epilogue for all other types other for YAML archives
  template <class T, traits::EnableIf<!std::is_arithmetic<T>::value,
                                      !traits::has_minimal_base_class_serialization<T, traits::has_minimal_input_serialization, YAMLInputArchive>::value,
                                      !traits::has_minimal_input_serialization<T, YAMLInputArchive>::value> = traits::sfinae>
  inline void epilogue( YAMLInputArchive & ar, T const & )
  {
    ar.finishNode();
  }

  // ######################################################################
  //! Prologue for arithmetic types for YAML archives
  inline
  void prologue( YAMLOutputArchive & ar, std::nullptr_t const & )
  {
    ar.writeName();
  }

  //! Prologue for arithmetic types for YAML archives
  inline
  void prologue( YAMLInputArchive &, std::nullptr_t const & )
  { }

  // ######################################################################
  //! Epilogue for arithmetic types for YAML archives
  inline
  void epilogue( YAMLOutputArchive &, std::nullptr_t const & )
  { }

  //! Epilogue for arithmetic types for YAML archives
  inline
  void epilogue( YAMLInputArchive &, std::nullptr_t const & )
  { }

  // ######################################################################
  //! Prologue for arithmetic types for YAML archives
  template <class T, traits::EnableIf<std::is_arithmetic<T>::value> = traits::sfinae> inline
  void prologue( YAMLOutputArchive & ar, T const & )
  {
    ar.writeName();
  }

  //! Prologue for arithmetic types for YAML archives
  template <class T, traits::EnableIf<std::is_arithmetic<T>::value> = traits::sfinae> inline
  void prologue( YAMLInputArchive &, T const & )
  { }

  // ######################################################################
  //! Epilogue for arithmetic types for YAML archives
  template <class T, traits::EnableIf<std::is_arithmetic<T>::value> = traits::sfinae> inline
  void epilogue( YAMLOutputArchive  &, T const & )
  { }

  //! Epilogue for arithmetic types for YAML archives
  template <class T, traits::EnableIf<std::is_arithmetic<T>::value> = traits::sfinae> inline
  void epilogue( YAMLInputArchive &, T const & )
  { }

  // ######################################################################
  //! Prologue for strings for YAML archives
  template<class CharT, class Traits, class Alloc> inline
  void prologue(YAMLOutputArchive & ar, std::basic_string<CharT, Traits, Alloc> const &)
  {
    ar.writeName();
    ar.putStringsQuoted();
  }

  //! Prologue for strings for YAML archives
  template<class CharT, class Traits, class Alloc> inline
  void prologue(YAMLInputArchive &, std::basic_string<CharT, Traits, Alloc> const &)
  { }

  // ######################################################################
  //! Epilogue for strings for YAML archives
  template<class CharT, class Traits, class Alloc> inline
  void epilogue(YAMLOutputArchive  &ar, std::basic_string<CharT, Traits, Alloc> const &)
  {
    ar.putStringsPlain();
  }

  //! Epilogue for strings for YAML archives
  template<class CharT, class Traits, class Alloc> inline
  void epilogue(YAMLInputArchive &, std::basic_string<CharT, Traits, Alloc> const &)
  { }

  // ######################################################################
  // Common YAMLArchive serialization functions
  // ######################################################################
  //! Serializing NVP types to YAML
  template <class T> inline
  void CEREAL_SAVE_FUNCTION_NAME( YAMLOutputArchive  & ar, NameValuePair<T> const & t )
  {
    ar.setNextName( t.name );
    ar( t.value );
  }

  template <class T> inline
  void CEREAL_LOAD_FUNCTION_NAME( YAMLInputArchive & ar, NameValuePair<T> & t )
  {
    ar.setNextName( t.name );
    ar( t.value );
  }

  //! Saving for nullptr to YAML
  inline
  void CEREAL_SAVE_FUNCTION_NAME(YAMLOutputArchive  & ar, std::nullptr_t const & t)
  {
    ar.saveValue( t );
  }

  //! Loading arithmetic from YAML
  inline
  void CEREAL_LOAD_FUNCTION_NAME(YAMLInputArchive & ar, std::nullptr_t & t)
  {
    ar.loadValue( t );
  }

  //! Saving for arithmetic to YAML
  template <class T, traits::EnableIf<std::is_arithmetic<T>::value> = traits::sfinae> inline
  void CEREAL_SAVE_FUNCTION_NAME(YAMLOutputArchive & ar, T const & t)
  {
    ar.saveValue( t );
  }

  //! Loading arithmetic from YAML
  template <class T, traits::EnableIf<std::is_arithmetic<T>::value> = traits::sfinae> inline
  void CEREAL_LOAD_FUNCTION_NAME(YAMLInputArchive & ar, T & t)
  {
    ar.loadValue( t );
  }

  //! saving string to YAML
  template<class CharT, class Traits, class Alloc> inline
  void CEREAL_SAVE_FUNCTION_NAME(YAMLOutputArchive  & ar, std::basic_string<CharT, Traits, Alloc> const & str)
  {
    ar.saveValue( str );
  }

  //! loading string from YAML
  template<class CharT, class Traits, class Alloc> inline
  void CEREAL_LOAD_FUNCTION_NAME(YAMLInputArchive & ar, std::basic_string<CharT, Traits, Alloc> & str)
  {
    ar.loadValue( str );
  }

  // ######################################################################
  //! Saving SizeTags to YAML
  template <class T> inline
  void CEREAL_SAVE_FUNCTION_NAME( YAMLOutputArchive  &, SizeTag<T> const & )
  {
    // nothing to do here, we don't explicitly save the size
  }

  //! Loading SizeTags from YAML
  template <class T> inline
  void CEREAL_LOAD_FUNCTION_NAME( YAMLInputArchive & ar, SizeTag<T> & st )
  {
    ar.loadSize( st.size );
  }

} // namespace ce

// register archives for polymorphic support
CEREAL_REGISTER_ARCHIVE(ce::YAMLInputArchive)
CEREAL_REGISTER_ARCHIVE(ce::YAMLOutputArchive)

// tie input and output archives together
CEREAL_SETUP_ARCHIVE_TRAITS(ce::YAMLInputArchive, ce::YAMLOutputArchive)

#endif // CE_ARCIVES_YAML_HPP_
