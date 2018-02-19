<%@ include file="header.jsp" %>
<%@ include file="sidebar-user-executive.jsp" %>
<script language="Javascript">
function xmlhttpPost(strURL) {

//settings for browsers 

var xmlHttpReq = false;
var self = this;
// Mozilla/Safari
if (window.XMLHttpRequest) {
self.xmlHttpReq = new XMLHttpRequest();
}
// IE
else if (window.ActiveXObject) {
self.xmlHttpReq = new ActiveXObject("Microsoft.XMLHTTP");
}

//open the connection
self.xmlHttpReq.open('POST', strURL, true);

//setting header Info
self.xmlHttpReq.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');

//check the ready state 4 and call function for updation of data
//initially the state is 0. 
//when the data comes from jsp, the state changes to 4 
// and den the function update gets called. 
self.xmlHttpReq.onreadystatechange = function() {
if (self.xmlHttpReq.readyState == 4) {
updatepage(self.xmlHttpReq.responseText); 
}
}
//send the value given by user to jsp.
self.xmlHttpReq.send(getquerystring()); 
}

//getquerystring function
function getquerystring() {
var form = document.forms['myForm'];
var cat = form.category.value;
qstr = 'cat=' + escape(cat); // NOTE: no '?' before querystring
return qstr;
}

//update function
function updatepage(str){
document.getElementById("result").innerHTML = str;
}

</script>
 <section id="main-content">
   <section class="wrapper">
   <sql:query dataSource="${ds}" var="result">
									SELECT * from category;
									</sql:query> 
      <div class="row">
      <div class="alert alert-info fade in">
                                  <button data-dismiss="alert" class="close close-sm" type="button">
                                      <i class="icon-remove"></i>
                                  </button>
                                  Welcome <strong><%=session.getAttribute("email") %></strong>
                                 <div align="right"><a href="<%=request.getContextPath() %>/Admin?action=user-login">Log Out</a></div>
                              </div>
                  <div class="col-lg-6">
                      <section class="panel">
                          <header class="panel-heading">
                             <strong>Add Banner (Advert) </strong>                 </header>
                          <div class="panel-body">
                          <font color="red" ><strong><c:out value="${msg }"></c:out></strong></font>
                              <form name="myForm" method="post" action="<%=request.getContextPath() %>/Admin?action=addb" enctype="multipart/form-data">
                              
                                         
                                      
                                   <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Category <span class="required">*</span></label>
                                          <select name="category" onchange='JavaScript:xmlhttpPost("fetch.jsp")' class="form-control  m-bot13" >
											<option>--Select News Type--</option>
											<c:forEach var="row" items="${result.rows}">
											<option value="<c:out value="${row.id}"/>"><c:out value="${row.name}"/></option>
											</c:forEach>
											</select>
                                      </div>
                                  <div id="result"> </div>
                                 </form>
                                      </div>
                                 
                                  </div>
                   
                      </section>
                  </div>
                  </div>
                  </section>
                  </section>
                  <%@ include file="footer.jsp" %>